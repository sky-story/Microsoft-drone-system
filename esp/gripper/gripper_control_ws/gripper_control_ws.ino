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

// ===== WebSocket =====
WebSocketsServer ws(81);

// ===== Servo =====
Servo servo;
const int servoPin = 18;

// Attach range (safe defaults; keep)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// ---- CALIBRATE THESE 3 VALUES ----
// Start with these, then adjust based on your real gripper geometry.
int HOLD_US    = 1500; // your "0° hold" reference
int RELEASE_US = 1900; // your "-100° from hold" (more open)
int GRIP_US    = 600; // your "120°" (closed/grip)

// Sweep speed for "grip" action
const int SWEEP_STEP_US = 10;     // smaller = smoother
const int SWEEP_DELAY_MS = 10;   // smaller = faster

void setServoUs(int us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(us);
  Serial.print("Servo us = ");
  Serial.println(us);
}

void sweepServoUs(int fromUs, int toUs) {
  fromUs = constrain(fromUs, SERVO_MIN_US, SERVO_MAX_US);
  toUs   = constrain(toUs,   SERVO_MIN_US, SERVO_MAX_US);

  if (fromUs < toUs) {
    for (int u = fromUs; u <= toUs; u += SWEEP_STEP_US) {
      setServoUs(u);
      delay(SWEEP_DELAY_MS);
      ws.loop(); // keep WS responsive during sweep (simple hack)
    }
  } else {
    for (int u = fromUs; u >= toUs; u -= SWEEP_STEP_US) {
      setServoUs(u);
      delay(SWEEP_DELAY_MS);
      ws.loop();
    }
  }
}

void sendHelp(uint8_t clientNum) {
  String msg =
    "cmds: hold | release | grip | status\n";
  ws.sendTXT(clientNum, msg);
}

void handleCommand(const String& raw, uint8_t clientNum) {
  String cmd = raw;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "hold") {
    setServoUs(HOLD_US);
    ws.sendTXT(clientNum, "ok: hold");
    return;
  }

  if (cmd == "release") {
    setServoUs(RELEASE_US);
    ws.sendTXT(clientNum, "ok: release");
    return;
  }

  if (cmd == "grip") {
    // “grip will start from -100 to 120 degree”
    // implement as sweep from RELEASE_US to GRIP_US
    sweepServoUs(RELEASE_US, GRIP_US);
    ws.sendTXT(clientNum, "ok: grip (sweep done)");
    return;
  }

  if (cmd == "status") {
    String s = "HOLD_US=" + String(HOLD_US) +
               " RELEASE_US=" + String(RELEASE_US) +
               " GRIP_US=" + String(GRIP_US);
    ws.sendTXT(clientNum, s);
    return;
  }

  // Optional: quick tuning commands from terminal, e.g. "set:hold:1520"
  if (cmd.startsWith("set:")) {
    // format: set:hold:1500  set:release:1200  set:grip:1900
    int p1 = cmd.indexOf(':');
    int p2 = cmd.indexOf(':', p1 + 1);
    if (p2 > 0) {
      String which = cmd.substring(p1 + 1, p2);
      int val = cmd.substring(p2 + 1).toInt();
      if (which == "hold") HOLD_US = val;
      else if (which == "release") RELEASE_US = val;
      else if (which == "grip") GRIP_US = val;
      ws.sendTXT(clientNum, "ok: " + cmd);
      return;
    }
  }

  ws.sendTXT(clientNum, "err: unknown");
  sendHelp(clientNum);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = ws.remoteIP(num);
    Serial.printf("WS client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
    // Single welcome message so client's first recv() after sending a command gets "ok: ..." not help text
    ws.sendTXT(num, "connected. cmds: hold | release | grip | status");
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WS client #%u disconnected\n", num);
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
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);

  servo.setPeriodHertz(50);
  servo.attach(servoPin, SERVO_MIN_US, SERVO_MAX_US);
  setServoUs(HOLD_US);

  connectWiFi();

  ws.begin();
  ws.onEvent(onWsEvent);

  Serial.println("WS server: ws://<ESP32_IP>:81");
}

void loop() {
  ws.loop();
}
