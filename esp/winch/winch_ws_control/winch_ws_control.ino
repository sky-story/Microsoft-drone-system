#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ====== WiFi #1: ANAFI ======
const char* ssid1 = "ANAFI Ai 001699";
const char* password1 = "HK433JUKWP53";
IPAddress local_IP1(192, 168, 42, 40);
IPAddress gateway1(192, 168, 42, 1);
IPAddress subnet1(255, 255, 255, 0);

// ====== WiFi #2: Itzsyboo ======
const char* ssid2 = "Itzsyboo";
const char* password2 = "0955399980";
IPAddress local_IP2(172, 20, 10, 2);
IPAddress gateway2(172, 20, 10, 1);
IPAddress subnet2(255, 255, 255, 0);

// ====== DNS (optional) ======
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(8, 8, 4, 4);

// ====== Servo / Winch ======
Servo servo;
const int servoPin = 18;

const int STOP_US  = 1500;
const int PULL_US  = 1700; // up
const int LOWER_US = 1300; // down

// ====== Hall sensor ======
const int hallPin = 19;                 // pick a GPIO that supports interrupts
const int PULSES_PER_REV = 1;           // start with 1 magnet -> 1 pulse per rev
const float SPOOL_DIAMETER_MM = 15.7f;
const float CIRC_MM = 3.1415926f * SPOOL_DIAMETER_MM; // ~49.32mm per rev

volatile long pulseCount = 0;           // total pulses since last zero
volatile unsigned long lastPulseUs = 0; // for debounce
const unsigned long DEBOUNCE_US = 3000; // 3ms debounce (tune if noisy)

// ====== Limit switch (Top) ======
const int topLimitPin = 5;             // choose a free GPIO that supports interrupts
volatile bool topHit = false;
volatile unsigned long lastTopUs = 0;
const unsigned long TOP_DEBOUNCE_US = 5000; // 5ms debounce (tune if noisy)

// ====== Solenoid (MOSFET low-side) ======
const int solenoidPin = 17; // change if you want another GPIO (avoid strapping pins)
bool solenoidState = false; // cached state
unsigned long solenoidPulseEndMs = 0; // non-blocking pulse timeout (0 = no pulse scheduled)
const unsigned long SOLENOID_UNLOCK_DELAY_MS = 500;

// ====== Motion control state ======
enum Mode { IDLE, LOWERING, PULLING };
Mode mode = IDLE;

long targetPulses = 0; // pulses to move for current command

// WebSocket: notify this client when move completes (target reached or top limit)
uint8_t notifyClientNum = 0;
bool notifyWhenDone = false;

// WebSocket server on port 81
WebSocketsServer ws(81);

// ---------- ISRs ----------
void IRAM_ATTR onHallPulse() {
  unsigned long now = micros();
  if (now - lastPulseUs > DEBOUNCE_US) {
    pulseCount++;
    lastPulseUs = now;
  }
}

void IRAM_ATTR onTopLimit() {
  unsigned long now = micros();
  if (now - lastTopUs > TOP_DEBOUNCE_US) {
    topHit = true;
    lastTopUs = now;
  }
}

// ---------- Helpers ----------
void setServoUs(int us) {
  servo.writeMicroseconds(us);
  Serial.print("Servo = ");
  Serial.println(us);
}

void setSolenoid(bool on) {
  digitalWrite(solenoidPin, on ? HIGH : LOW);
  solenoidState = on;
  if (!on) solenoidPulseEndMs = 0;
  Serial.print("Solenoid -> ");
  Serial.println(on ? "ON" : "OFF");
}

void pulseSolenoid(unsigned long ms) {
  if (ms == 0) return;
  setSolenoid(true);
  solenoidPulseEndMs = millis() + ms;
  Serial.printf("Solenoid pulsed for %lums\n", ms);
}

void stopMotor() {
  setServoUs(STOP_US);
  mode = IDLE;
  targetPulses = 0;
  setSolenoid(false);
}

bool isTopPressed() {
  // Using INPUT_PULLUP: pressed = LOW (wired to GND)
  return digitalRead(topLimitPin) == LOW;
}

long mmToPulses(float mm) {
  // pulses = mm * (pulses/rev) / (mm/rev)
  //        = mm * PULSES_PER_REV / CIRC_MM
  long p = lround(mm * (float)PULSES_PER_REV / CIRC_MM);
  if (p < 0) p = 0;
  return p;
}

void startLowerMm(float mm) {
  if (mm <= 0) return;

  // Unlock before lowering
  setSolenoid(true);
  delay(SOLENOID_UNLOCK_DELAY_MS);

  noInterrupts();
  pulseCount = 0;
  interrupts();

  targetPulses = mmToPulses(mm);
  Serial.print("Lower target mm = ");
  Serial.print(mm);
  Serial.print(" -> target pulses = ");
  Serial.println(targetPulses);

  if (targetPulses == 0) {
    Serial.println("Target pulses is 0 (too small).");
    return;
  }

  mode = LOWERING;
  setServoUs(LOWER_US);
}

void startPullMm(float mm) {
  if (mm <= 0) return;
  if (isTopPressed()) {
    Serial.println("Already at TOP -> block pull");
    stopMotor();
    return;
  }

  // Unlock before pulling
  setSolenoid(true);
  delay(SOLENOID_UNLOCK_DELAY_MS);

  noInterrupts();
  topHit = false;
  pulseCount = 0;
  interrupts();

  targetPulses = mmToPulses(mm);
  Serial.print("Pull target mm = ");
  Serial.print(mm);
  Serial.print(" -> target pulses = ");
  Serial.println(targetPulses);

  if (targetPulses == 0) {
    Serial.println("Target pulses is 0 (too small).");
    return;
  }

  mode = PULLING;
  setServoUs(PULL_US);
}

void handleCommand(const String& cmdRaw, uint8_t clientNum) {
  String cmd = cmdRaw;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "stop") {
    notifyWhenDone = false;
    stopMotor();
    ws.sendTXT(clientNum, "ok: stop");
    return;
  }

  if (cmd == "zero") {
    noInterrupts();
    pulseCount = 0;
    interrupts();
    ws.sendTXT(clientNum, "ok: zero");
    return;
  }

  // lower:500  (mm)
  if (cmd.startsWith("lower:")) {
    float mm = cmd.substring(6).toFloat();
    startLowerMm(mm);
    if (mode == LOWERING && targetPulses > 0) {
      notifyClientNum = clientNum;
      notifyWhenDone = true;
    }
    ws.sendTXT(clientNum, "ok: lower " + String(mm) + "mm");
    return;
  }

  // pull:200 (mm)
  if (cmd.startsWith("pull:")) {
    float mm = cmd.substring(5).toFloat();
    startPullMm(mm);
    if (mode == PULLING && targetPulses > 0) {
      notifyClientNum = clientNum;
      notifyWhenDone = true;
    }
    if (isTopPressed()) {
      ws.sendTXT(clientNum, "err: top limit pressed");
    } else {
      ws.sendTXT(clientNum, "ok: pull " + String(mm) + "mm");
    }
    return;
  }

  // optional direct mode (continuous)
  if (cmd == "lower") {
    mode = LOWERING;
    setServoUs(LOWER_US);
    ws.sendTXT(clientNum, "ok: lower (continuous)");
    return;
  }

  if (cmd == "pull") {
    if (isTopPressed()) {
      stopMotor();
      ws.sendTXT(clientNum, "err: top limit pressed");
      return;
    }
    // clear topHit for this continuous pull session
    noInterrupts();
    topHit = false;
    interrupts();

    mode = PULLING;
    setServoUs(PULL_US);
    ws.sendTXT(clientNum, "ok: pull (continuous)");
    return;
  }

  // ---- Solenoid commands ----
  // sol:on
  // sol:off
  // sol:pulse:MS  (example sol:pulse:150 => 150 ms)
  if (cmd == "sol:on" || cmd == "sol:on\r" || cmd == "sol:on\n") {
    setSolenoid(true);
    ws.sendTXT(clientNum, "ok: solenoid on");
    return;
  }
  if (cmd == "sol:off" || cmd == "sol:off\r" || cmd == "sol:off\n") {
    setSolenoid(false);
    ws.sendTXT(clientNum, "ok: solenoid off");
    return;
  }
  if (cmd.startsWith("sol:pulse:")) {
    long ms = cmd.substring(10).toInt();
    if (ms <= 0) {
      ws.sendTXT(clientNum, "err: invalid pulse ms");
    } else {
      pulseSolenoid((unsigned long)ms);
      ws.sendTXT(clientNum, "ok: solenoid pulse " + String(ms) + "ms");
    }
    return;
  }

  ws.sendTXT(clientNum, "err: unknown command");
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = ws.remoteIP(num);
    Serial.printf("WS client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
    ws.sendTXT(num, "connected. cmds: lower:MM pull:MM stop zero sol:on sol:off sol:pulse:MS");
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WS client #%u disconnected -> stop\n", num);
    stopMotor(); // safety
  } else if (type == WStype_TEXT) {
    String msg = String((char*)payload).substring(0, length);
    Serial.printf("WS msg: %s\n", msg.c_str());
    handleCommand(msg, num);
  }
}

void connectWiFi(const char* ssid, const char* password,
                 IPAddress local_IP, IPAddress gateway, IPAddress subnet) {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  if (!WiFi.config(local_IP, gateway, subnet, dns1, dns2)) {
    Serial.println("⚠️  Static IP config failed!");
  }

  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();

  // Try up to 10 seconds
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nConnection failed.");
  }
}


void setup() {
  Serial.begin(115200);  // Match Serial Monitor baud (74880 caused garbled output)

  // Servo
  servo.setPeriodHertz(50);
  servo.attach(servoPin, 500, 2500);
  stopMotor();

  // Hall sensor
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, FALLING); // or RISING depending sensor

  // Top limit switch
  pinMode(topLimitPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(topLimitPin), onTopLimit, FALLING); // pressed -> LOW

  // Solenoid pin
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW); // default OFF

  // WiFi + WS
  // Try ANAFI first
  connectWiFi(ssid1, password1, local_IP1, gateway1, subnet1);

  // If not connected, try Itzsyboo
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(ssid2, password2, local_IP2, gateway2, subnet2);
  }

  // Final status
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected successfully.");
  } else {
    Serial.println("Unable to connect to any WiFi.");
  }
  ws.begin();
  ws.onEvent(onWsEvent);

  Serial.println("WS: ws://<ESP32_IP>:81");
  Serial.println("Commands: lower:500  pull:200  stop  zero  sol:on sol:off sol:pulse:MS");
  Serial.println("Top limit: stops pulling immediately when pressed.");
}

void loop() {
  ws.loop();

  // handle solenoid pulse timeout (non-blocking)
  if (solenoidPulseEndMs != 0 && millis() >= solenoidPulseEndMs) {
    setSolenoid(false);
    solenoidPulseEndMs = 0;
    Serial.println("Solenoid pulse ended -> OFF");
  }

  // Immediate stop if top limit is triggered while pulling
  if (mode == PULLING) {
    bool hit;
    noInterrupts();
    hit = topHit;
    interrupts();

    if (hit) {
      Serial.println("TOP LIMIT HIT -> stop");
      if (notifyWhenDone) {
        ws.sendTXT(notifyClientNum, "ok: done");
        notifyWhenDone = false;
      }
      stopMotor();

      // clear flag so it doesn't keep stopping forever
      noInterrupts();
      topHit = false;
      interrupts();
    }
  }

  // Auto-stop when target pulses reached
  if (mode == LOWERING || mode == PULLING) {
    long pc;
    noInterrupts();
    pc = pulseCount;
    interrupts();

    if (targetPulses > 0 && pc >= targetPulses) {
      Serial.print("Target reached pulses=");
      Serial.println(pc);
      if (notifyWhenDone) {
        ws.sendTXT(notifyClientNum, "ok: done");
        notifyWhenDone = false;
      }
      stopMotor();
    }
  }
}