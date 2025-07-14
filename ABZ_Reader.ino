#include <Arduino.h>

// ——— Pin Definitions ———
const int ENC_A_PIN = 15;
const int ENC_B_PIN = 2;
const int ENC_Z_PIN = 14;

// ——— Encoder State ———
volatile long encCount = 0;
volatile int8_t lastEnc = 0;

// ——— Initial calibration ———
int ppr = 20;  // Pulses per revolution

// ——— Timing ———
unsigned long lastPlot = 0;
const unsigned long PLOT_INTERVAL = 20; // ms

// ——— Flags ———
bool showAngle = false;
bool showRawSignals = false;
bool selfCalibrate = false;

// ——— ISRs ———
void IRAM_ATTR handleA() {
  int8_t A = digitalRead(ENC_A_PIN);
  int8_t B = digitalRead(ENC_B_PIN);
  int8_t curr = (A << 1) | B;
  int8_t sum = (lastEnc << 2) | curr;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encCount--;
  lastEnc = curr;
}
void IRAM_ATTR handleB() {
  handleA();
}
void IRAM_ATTR handleZ() {
  // Optional: could reset encoder count
}

// ——— Setup ———
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_Z_PIN, INPUT_PULLUP);

  lastEnc = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), handleB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z_PIN), handleZ, RISING);

  showMenu();
}

// ——— Helper Functions ———
void showMenu() {
  Serial.println("\n=== ENCODER TOOL ===");
  Serial.println("Type:");
  Serial.println("  N     → Normal angle tracking mode");
  Serial.println("  D     → Digital signal monitor (A/B/Z)");
  Serial.println("  S     → Self-calibration (make 1 full turn)");
  Serial.println("  <num> → Set PPR directly (e.g., 4096)");
  Serial.println("  M     → Show this menu again");
  Serial.println("====================\n");
}

void runSelfCalibration() {
  Serial.println("[Calibration] Turn exactly 1 full revolution then type any key.");

  encCount = 0;
  while (!Serial.available()) {
    delay(10);
  }
  Serial.read(); // clear input
  long pulses;
  noInterrupts(); pulses = encCount; interrupts();

  int estimate = abs(pulses) / 4;
  Serial.print("[Calibration] Approximate PPR: ");
  Serial.println(estimate);
  ppr = estimate;
  selfCalibrate = false;
  showAngle = false;
  showRawSignals = false;
}

// ——— Main Loop ———
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("n")) {
      showAngle = true;
      showRawSignals = false;
      selfCalibrate = false;
      Serial.println("[Mode] Angle tracking mode");
    } else if (cmd.equalsIgnoreCase("d")) {
      showRawSignals = true;
      showAngle = false;
      selfCalibrate = false;
      Serial.println("[Mode] Digital pin monitor");
    } else if (cmd.equalsIgnoreCase("s")) {
      selfCalibrate = true;
      showRawSignals = false;
      showAngle = false;
    } else if (cmd.equalsIgnoreCase("m")) {
      showMenu();
    } else if (cmd.toInt() > 0) {
      ppr = cmd.toInt();
      Serial.print("[PPR Updated] Now using: ");
      Serial.println(ppr);
    }
  }

  if (selfCalibrate) {
    runSelfCalibration();
  }

  unsigned long now = millis();
  if (now - lastPlot >= PLOT_INTERVAL) {
    lastPlot = now;

    if (showAngle) {
      long cnt;
      noInterrupts(); cnt = encCount; interrupts();
      float deg = (cnt / float(ppr)) * 360.0;
      Serial.println(deg);
    }

    if (showRawSignals) {
      Serial.print("A: ");
      Serial.print(digitalRead(ENC_A_PIN));
      Serial.print(" | B: ");
      Serial.print(digitalRead(ENC_B_PIN));
      Serial.print(" | Z: ");
      Serial.println(digitalRead(ENC_Z_PIN));
    }
  }
}
