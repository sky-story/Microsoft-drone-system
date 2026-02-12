#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

Servo servo;
const int servoPin = 18;

// ====== WiFi ======
// IP -> ws://192.168.42.39:81
const char* ssid = "ANAFI Ai 001699";
const char* password = "HK433JUKWP53";

// IP -> ws://172.20.10.3:81
// const char* ssid = "Itzsyboo";
// const char* password = "0955399980";

// --- Servo microseconds (tune if needed) ---
const int STOP_US  = 1500;
const int PULL_US  = 1700; // one direction
const int LOWER_US = 1300; // other direction

WebServer server(80);

void setServoUs(int us) {
  servo.writeMicroseconds(us);
  Serial.print("Servo set to ");
  Serial.println(us);
}

void handleRoot() {
  String msg =
    "ESP32 Winch Control\n"
    "Commands:\n"
    "  /pull  -> run one direction\n"
    "  /lower -> run other direction\n"
    "  /stop  -> neutral\n";
  server.send(200, "text/plain", msg);
}

void handlePull() {
  setServoUs(PULL_US);
  server.send(200, "text/plain", "PULL");
}

void handleLower() {
  setServoUs(LOWER_US);
  server.send(200, "text/plain", "LOWER");
}

void handleStop() {
  setServoUs(STOP_US);
  server.send(200, "text/plain", "STOP");
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) { // ~20s
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed (check SSID/pass or 2.4GHz).");
  }
}

void setup() {
  Serial.begin(115200);

  // Servo init
  servo.setPeriodHertz(50);
  servo.attach(servoPin, 500, 2500);
  setServoUs(STOP_US);

  // WiFi init
  connectWiFi();

  // Routes
  server.on("/", handleRoot);
  server.on("/pull", handlePull);
  server.on("/lower", handleLower);
  server.on("/stop", handleStop);

  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Try: http://<ESP32_IP>/pull  /lower  /stop");
}

void loop() {
  server.handleClient();
}

