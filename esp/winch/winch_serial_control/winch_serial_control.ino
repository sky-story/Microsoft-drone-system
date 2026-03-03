#include <ESP32Servo.h>

// ====== Servo / Winch ======
Servo servo;
const int servoPin = 18;

const int STOP_US  = 1500;
const int PULL_US  = 1700; // up
const int LOWER_US = 1300; // down

// ====== Hall sensor ======
const int hallPin = 19;
const int PULSES_PER_REV = 1;
const float SPOOL_DIAMETER_MM = 15.7f;
const float CIRC_MM = 3.1415926f * SPOOL_DIAMETER_MM;

volatile long pulseCount = 0;
volatile unsigned long lastPulseUs = 0;
const unsigned long DEBOUNCE_US = 3000;

// ====== Limit switch (Top) ======
const int topLimitPin = 5;
volatile bool topHit = false;
volatile unsigned long lastTopUs = 0;
const unsigned long TOP_DEBOUNCE_US = 5000;

// ====== Solenoid ======
const int solenoidPin = 17;
bool solenoidState = false;
unsigned long solenoidPulseEndMs = 0;
const unsigned long SOLENOID_UNLOCK_DELAY_MS = 500;

// ====== Motion control ======
enum Mode { IDLE, LOWERING, PULLING };
Mode mode = IDLE;
long targetPulses = 0;

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
  return digitalRead(topLimitPin) == LOW; // pressed = LOW
}

long mmToPulses(float mm) {
  long p = lround(mm * (float)PULSES_PER_REV / CIRC_MM);
  if (p < 0) p = 0;
  return p;
}

void startLowerMm(float mm) {
  if (mm <= 0) return;
  setSolenoid(true);
  delay(SOLENOID_UNLOCK_DELAY_MS);

  noInterrupts();
  pulseCount = 0;
  interrupts();

  targetPulses = mmToPulses(mm);
  Serial.printf("Lower %.1f mm -> %ld pulses\n", mm, targetPulses);

  if (targetPulses == 0) return;
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

  setSolenoid(true);
  delay(SOLENOID_UNLOCK_DELAY_MS);

  noInterrupts();
  topHit = false;
  pulseCount = 0;
  interrupts();

  targetPulses = mmToPulses(mm);
  Serial.printf("Pull %.1f mm -> %ld pulses\n", mm, targetPulses);

  if (targetPulses == 0) return;
  mode = PULLING;
  setServoUs(PULL_US);
}

// ---------- Serial Command Parser ----------
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "stop") {
    stopMotor();
    Serial.println("ok: stop");
    return;
  }

  if (cmd == "zero") {
    noInterrupts();
    pulseCount = 0;
    interrupts();
    Serial.println("ok: zero");
    return;
  }

  if (cmd.startsWith("lower:")) {
    float mm = cmd.substring(6).toFloat();
    startLowerMm(mm);
    Serial.printf("ok: lower %.1f mm\n", mm);
    return;
  }

  if (cmd.startsWith("pull:")) {
    float mm = cmd.substring(5).toFloat();
    startPullMm(mm);
    if (isTopPressed()) Serial.println("err: top limit pressed");
    else Serial.printf("ok: pull %.1f mm\n", mm);
    return;
  }

  if (cmd == "lower") {
    mode = LOWERING;
    setServoUs(LOWER_US);
    Serial.println("ok: lower (continuous)");
    return;
  }

  if (cmd == "pull") {
    if (isTopPressed()) {
      stopMotor();
      Serial.println("err: top limit pressed");
      return;
    }
    noInterrupts();
    topHit = false;
    interrupts();
    mode = PULLING;
    setServoUs(PULL_US);
    Serial.println("ok: pull (continuous)");
    return;
  }

  if (cmd == "sol:on") {
    setSolenoid(true);
    Serial.println("ok: solenoid on");
    return;
  }
  if (cmd == "sol:off") {
    setSolenoid(false);
    Serial.println("ok: solenoid off");
    return;
  }
  if (cmd.startsWith("sol:pulse:")) {
    long ms = cmd.substring(10).toInt();
    if (ms > 0) {
      pulseSolenoid(ms);
      Serial.printf("ok: solenoid pulse %ld ms\n", ms);
    } else Serial.println("err: invalid pulse ms");
    return;
  }

  Serial.println("err: unknown command");
}

void setup() {
  Serial.begin(115200);       // USB Serial
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // optional: RX=16, TX=17 (for wired control)

  servo.setPeriodHertz(50);
  servo.attach(servoPin, 500, 2500);
  stopMotor();

  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, FALLING);

  pinMode(topLimitPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(topLimitPin), onTopLimit, FALLING);

  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);

  Serial.println("Winch ready. Commands:");
  Serial.println("lower:MM pull:MM stop zero sol:on sol:off sol:pulse:MS");
  Serial.println("Use USB Serial or Serial2 to send commands.");
}

void loop() {
  // ---- Read Serial (USB) commands ----
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // ---- Read Serial2 commands (optional) ----
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    handleCommand(cmd);
  }

  // ---- Solenoid pulse timeout ----
  if (solenoidPulseEndMs != 0 && millis() >= solenoidPulseEndMs) {
    setSolenoid(false);
    solenoidPulseEndMs = 0;
    Serial.println("Solenoid pulse ended -> OFF");
  }

  // ---- Top limit stop ----
  if (mode == PULLING) {
    bool hit;
    noInterrupts();
    hit = topHit;
    interrupts();

    if (hit) {
      Serial.println("TOP LIMIT HIT -> stop");
      stopMotor();
      noInterrupts();
      topHit = false;
      interrupts();
    }
  }

  // ---- Auto stop on distance ----
  if (mode == LOWERING || mode == PULLING) {
    long pc;
    noInterrupts();
    pc = pulseCount;
    interrupts();

    if (targetPulses > 0 && pc >= targetPulses) {
      Serial.printf("Target reached: %ld pulses\n", pc);
      stopMotor();
    }
  }
}
