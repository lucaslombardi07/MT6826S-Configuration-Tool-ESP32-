// ================= MT6826S SPI Tool (ESP32 DevKit V1) =================
// Encoder powered at 3.3 V
// Board row pins:  D14‑SCK | D13‑MISO | D12‑MOSI | D27‑CSN
// =====================================================================

#include <Arduino.h>
#include <SPI.h>

// ---------- Pin map ---------------------------------------------------
constexpr uint8_t PIN_SCK  = 14;   // D14
constexpr uint8_t PIN_MISO = 13;   // D13 (SDO ↔ ESP32 in)
constexpr uint8_t PIN_MOSI = 12;   // D12 (SDI ↔ ESP32 out)
constexpr uint8_t PIN_CS   = 27;   // D27 (active‑LOW)

// ---------- MT6826S register map bits ---------------------------------
constexpr uint16_t REG_ABZ_RES_MSB = 0x007;   // ABZ_RES[11:4] (bits 11:4 of PPR)
constexpr uint16_t REG_ABZ_RES_LSB = 0x008;   // bits 7:4 = ABZ_RES[3:0], bit1=ABZ_OFF, bit0=AB_SWAP
constexpr uint16_t REG_CFG_LOAD    = 0x011;   // pulse reload
constexpr uint16_t REG_STATUS      = 0x112;   // has EE_DONE flag, good probe reg
constexpr uint8_t  EE_DONE_BIT     = 0x20;    // bit‑5 of 0x112
constexpr uint16_t MAX_REG         = 0x12F;   // highest valid addr

// ===== forward declarations =====
uint8_t  readReg(uint16_t addr);
void     writeReg(uint16_t addr, uint8_t data);
void     fullDump();
void     setPPR(uint16_t ppr);
bool     verifyPPR(uint16_t ppr);
bool     programEEPROM();
uint16_t askForPPR();
void     showMenu();
void     forceABZOn();
void     getParameters();

// ====================== SET‑UP =======================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  delay(200);

  Serial.println("\nWaiting for MT6826S...");
  uint8_t stat = readReg(REG_STATUS);
  while (stat == 0x00 || stat == 0xFF) {
    Serial.print('.');
    delay(250);
    stat = readReg(REG_STATUS);
  }
  Serial.printf("\n[OK] Device online. STATUS=0x%02X\n", stat);

  showMenu();
}

void loop() {}

// ====================== MENU =========================================
void showMenu() {
  while (true) {
    Serial.println("\n===== MT6826S MENU =====");
    Serial.println("[1] Dump all registers");
    Serial.println("[2] Set PPR (ABZ_RES) — volatile");
    Serial.println("[3] Save current config to EEPROM");
    Serial.println("[4] Force ABZ Output ON (clear ABZ_OFF)");
    Serial.println("[5] Get Parameters");
    Serial.println("[q] Quit");
    Serial.print("Enter choice: ");

    while (!Serial.available()) delay(10);
    char c = Serial.read();
    Serial.readStringUntil('\n');   // clear rest of line

    if      (c == '1') fullDump();
    else if (c == '2') { uint16_t p = askForPPR(); setPPR(p); verifyPPR(p); }
    else if (c == '3') programEEPROM();
    else if (c == '4') forceABZOn();
    else if (c == '5') getParameters();
    else if (c == 'q') { Serial.println("Exiting menu."); return; }
    else               Serial.println("[!] Invalid option.");
  }
}

// ====================== LOW‑LEVEL SPI ================================
uint8_t readReg(uint16_t addr) {
  uint8_t val;
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x30 | ((addr >> 8) & 0x0F));
  SPI.transfer(addr & 0xFF);
  val = SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void writeReg(uint16_t addr, uint8_t data) {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x60 | ((addr >> 8) & 0x0F));
  SPI.transfer(addr & 0xFF);
  SPI.transfer(data);
  SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

// ====================== HIGH‑LEVEL OPS ===============================
void fullDump() {
  Serial.println("\n=== REGISTER DUMP ===");
  for (uint16_t a = 0; a <= MAX_REG; a++) {
    Serial.printf("0x%03X = 0x%02X\n", a, readReg(a));
    delay(1);
  }
}

void setPPR(uint16_t ppr) {
  if (ppr < 1 || ppr > 4096) {
    Serial.println("[!] PPR out of range 1‑4096");
    return;
  }

  uint16_t abz_res = ppr - 1; // 0 = 1PPR, 0xFFF = 4096
  uint8_t  msb = (abz_res >> 4) & 0xFF;        // ABZ_RES[11:4]
  uint8_t  lsb_old = readReg(REG_ABZ_RES_LSB);
  uint8_t  lsb = (lsb_old & 0x0F) | ((abz_res & 0x0F) << 4);  // ABZ_RES[3:0] in bits 7:4

  writeReg(REG_ABZ_RES_MSB, msb);
  writeReg(REG_ABZ_RES_LSB, lsb);
  writeReg(REG_CFG_LOAD, 0x01);
  writeReg(REG_CFG_LOAD, 0x00);

  Serial.printf("ABZ_RES set to %u PPR (encoded 0x%03X)\n", ppr, abz_res);
}

void forceABZOn() {
  uint8_t lsb = readReg(REG_ABZ_RES_LSB);
  lsb &= ~(1 << 1);  // clear ABZ_OFF (bit 1)
  writeReg(REG_ABZ_RES_LSB, lsb);
  writeReg(REG_CFG_LOAD, 0x01);
  writeReg(REG_CFG_LOAD, 0x00);
  Serial.printf("[INFO] ABZ_OFF cleared. LSB=0x%02X\n", lsb);
}

bool verifyPPR(uint16_t ppr) {
  uint8_t msb = readReg(REG_ABZ_RES_MSB);
  uint8_t lsb = readReg(REG_ABZ_RES_LSB);
  uint16_t val = ((msb << 4) | ((lsb >> 4) & 0x0F));
  uint16_t read_ppr = val + 1;
  bool ok = (read_ppr == ppr);
  Serial.printf("Verify: read 0x%03X → %u PPR → %s\n", val, read_ppr, ok ? "OK" : "MISMATCH");
  return ok;
}

bool programEEPROM() {
  uint8_t ack = 0;

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(0xC0);  // EEPROM program command
  SPI.transfer(0x00);  // Padding (3 bytes total)
  SPI.transfer(0x00);
  ack = SPI.transfer(0x00); // Read acknowledge
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  Serial.printf("EEPROM ack = 0x%02X\n", ack);
  if (ack != 0x55) {
    Serial.println("[!] EEPROM command failed.");
    return false;
  }

  Serial.print("Waiting for EEPROM write...");
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    if (readReg(REG_STATUS) & EE_DONE_BIT) {
      Serial.println(" done.");
      return true;
    }
    delay(10);
  }
  Serial.println(" timeout... It may have worked thought...");
  return false;
}

uint16_t askForPPR() {
  Serial.print("Enter new PPR (1‑4096): ");
  while (!Serial.available()) delay(10);
  uint16_t v = Serial.readStringUntil('\n').toInt();
  if (v < 1 || v > 4096) { Serial.println("[!] Invalid, default 4096"); v = 4096; }
  return v;
}

void getParameters() {
  uint8_t msb = readReg(REG_ABZ_RES_MSB);
  uint8_t lsb = readReg(REG_ABZ_RES_LSB);
  uint16_t val = ((msb << 4) | ((lsb >> 4) & 0x0F));
  uint16_t ppr = val + 1;
  bool abz_off = (lsb >> 1) & 0x01;
  bool ab_swap = (lsb >> 0) & 0x01;

  Serial.printf("\n--- Encoder Parameters ---\n");
  Serial.printf("PPR: %u\n", ppr);
  Serial.printf("ABZ Output: %s\n", abz_off ? "OFF" : "ON");
  Serial.printf("A/B Swap: %s\n", ab_swap ? "ENABLED" : "DISABLED");
}
