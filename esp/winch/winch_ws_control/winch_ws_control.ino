#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ====== WiFi ======
// IP -> ws://192.168.42.39:81
// const char* ssid = "ANAFI Ai 001699";
// const char* password = "HK433JUKWP53";

// IP -> ws://172.20.10.3:81
const char* ssid = "Itzsyboo";
const char* password = "0955399980";

// ====== Servo / Winch ======
Servo servo;
const int servoPin = 18;

const int STOP_US  = 1500;
const int PULL_US  = 1700; // up
const int LOWER_US = 1300; // down

// ====== Hall sensor ======
const int hallPin = 27;                 // pick a GPIO that supports interrupts
const int PULSES_PER_REV = 1;           // start with 1 magnet -> 1 pulse per rev
const float SPOOL_DIAMETER_MM = 15.7f;
const float CIRC_MM = 3.1415926f * SPOOL_DIAMETER_MM; // ~49.32mm per rev

volatile long pulseCount = 0;           // total pulses since last zero
volatile unsigned long lastPulseUs = 0; // for debounce
const unsigned long DEBOUNCE_US = 3000; // 3ms debounce (tune if noisy)

// ====== Limit switch (Top) ======
const int topLimitPin = 26;             // choose a free GPIO that supports interrupts
volatile bool topHit = false;
volatile unsigned long lastTopUs = 0;
const unsigned long TOP_DEBOUNCE_US = 5000; // 5ms debounce (tune if noisy)

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

void stopMotor() {
  setServoUs(STOP_US);
  mode = IDLE;
  targetPulses = 0;
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

  // If already at top, don't pull further
  if (isTopPressed()) {
    Serial.println("Already at TOP -> block pull");
    stopMotor();
    return;
  }

  // clear topHit flag for a new pull command
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

  ws.sendTXT(clientNum, "err: unknown command");
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = ws.remoteIP(num);
    Serial.printf("WS client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
    ws.sendTXT(num, "connected. cmds: lower:MM pull:MM stop zero");
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WS client #%u disconnected -> stop\n", num);
    stopMotor(); // safety
  } else if (type == WStype_TEXT) {
    String msg = String((char*)payload).substring(0, length);
    Serial.printf("WS msg: %s\n", msg.c_str());
    handleCommand(msg, num);
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed (2.4GHz SSID?)");
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

  // WiFi + WS
  connectWiFi();
  ws.begin();
  ws.onEvent(onWsEvent);

  Serial.println("WS: ws://<ESP32_IP>:81");
  Serial.println("Commands: lower:500  pull:200  stop  zero");
  Serial.println("Top limit: stops pulling immediately when pressed.");
}

void loop() {
  ws.loop();

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
