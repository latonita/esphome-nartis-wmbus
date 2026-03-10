// CMT2300A SPI Test — minimal bit-banged 3-wire SPI test for ESP32
// Verifies basic communication with CMT2300A radio module.
//
// Wiring (adjust to match your setup):
//   ESP32 GPIO13 -> SDIO (bidirectional data)
//   ESP32 GPIO14 -> SCK  (clock)
//   ESP32 GPIO27 -> CSB  (chip select)
//   ESP32 GPIO26 -> FCSB (FIFO chip select)
//   3.3V         -> VDD
//   GND          -> GND
//
// Upload via Arduino IDE with ESP32 board selected.
// Open Serial Monitor at 115200 baud.

#define PIN_SDIO  13
#define PIN_SCLK  14
#define PIN_CSB   27
#define PIN_FCSB  26

// CMT2300A registers
#define CMT2300A_CUS_MODE_CTL   0x60
#define CMT2300A_CUS_MODE_STA   0x61
#define CMT2300A_CUS_SOFTRST    0x7F
#define CMT2300A_GO_STBY        0x02

// --- Bit-banged 3-wire SPI ---

void spi_write_byte(uint8_t byte) {
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(PIN_SCLK, LOW);
    digitalWrite(PIN_SDIO, (byte >> i) & 1);
    delayMicroseconds(2);
    digitalWrite(PIN_SCLK, HIGH);
    delayMicroseconds(2);
  }
  digitalWrite(PIN_SCLK, LOW);
}

uint8_t spi_read_byte() {
  uint8_t byte = 0;
  pinMode(PIN_SDIO, INPUT);
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(PIN_SCLK, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SCLK, HIGH);
    if (digitalRead(PIN_SDIO))
      byte |= (1 << i);
    delayMicroseconds(2);
  }
  digitalWrite(PIN_SCLK, LOW);
  pinMode(PIN_SDIO, OUTPUT);
  return byte;
}

void write_reg(uint8_t addr, uint8_t value) {
  digitalWrite(PIN_CSB, LOW);
  delayMicroseconds(2);
  spi_write_byte(addr & 0x7F);  // bit7=0 for write
  spi_write_byte(value);
  digitalWrite(PIN_CSB, HIGH);
  delayMicroseconds(2);
}

uint8_t read_reg(uint8_t addr) {
  digitalWrite(PIN_CSB, LOW);
  delayMicroseconds(2);
  spi_write_byte(addr | 0x80);  // bit7=1 for read
  uint8_t val = spi_read_byte();
  digitalWrite(PIN_CSB, HIGH);
  delayMicroseconds(2);
  return val;
}

// --- Test ---

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n=== CMT2300A SPI Test ===\n");

  // Init pins
  pinMode(PIN_SDIO, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_CSB, OUTPUT);
  pinMode(PIN_FCSB, OUTPUT);

  digitalWrite(PIN_CSB, HIGH);
  digitalWrite(PIN_FCSB, HIGH);
  digitalWrite(PIN_SCLK, LOW);

  Serial.printf("Pins: SDIO=%d SCLK=%d CSB=%d FCSB=%d\n\n", PIN_SDIO, PIN_SCLK, PIN_CSB, PIN_FCSB);

  delay(10);

  // Step 1: Read reg 0x00 before reset (cold read)
  uint8_t cold = read_reg(0x00);
  Serial.printf("Step 1 - Cold read reg 0x00: 0x%02X\n", cold);
  if (cold == 0x00) Serial.println("  -> 0x00 = no response (SDIO stuck low or chip not connected)");
  else if (cold == 0xFF) Serial.println("  -> 0xFF = bus floating (SDIO not connected)");
  else Serial.printf("  -> non-zero = chip responding!\n");

  // Step 2: Soft reset
  Serial.println("\nStep 2 - Soft reset (write 0xFF to reg 0x7F)...");
  write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(20);

  // Step 3: Go to standby
  Serial.println("Step 3 - Go standby (write 0x02 to reg 0x60)...");
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  delay(20);

  // Step 4: Read mode status
  uint8_t mode_sta = read_reg(CMT2300A_CUS_MODE_STA);
  uint8_t chip_mode = mode_sta & 0x0F;
  Serial.printf("Step 4 - MODE_STA (reg 0x61): 0x%02X (mode bits: 0x%X)\n", mode_sta, chip_mode);
  if (chip_mode == 0x02) Serial.println("  -> STANDBY mode - chip is alive and responding!");
  else if (chip_mode == 0x01) Serial.println("  -> SLEEP mode - reset worked but standby command failed");
  else if (chip_mode == 0x00) Serial.println("  -> 0x00 = no response");
  else Serial.printf("  -> unexpected mode 0x%X\n", chip_mode);

  // Step 5: Read first 16 registers
  Serial.println("\nStep 5 - Register dump (0x00-0x0F):");
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t val = read_reg(i);
    Serial.printf("  reg[0x%02X] = 0x%02X%s\n", i, val, (val == 0x00) ? "  (zero)" : "");
  }

  // Step 6: Try swapped SDIO/SCLK hint
  bool all_zero = true;
  for (uint8_t i = 0; i < 16; i++) {
    if (read_reg(i) != 0x00) { all_zero = false; break; }
  }

  Serial.println("\n=== RESULT ===");
  if (all_zero) {
    Serial.println("FAIL: All registers read 0x00");
    Serial.println("Likely causes:");
    Serial.println("  1. SDIO and SCLK pins swapped");
    Serial.println("  2. CSB and FCSB pins swapped");
    Serial.println("  3. Wiring not connected properly");
    Serial.println("  4. Module not powered (check 3.3V on VDD)");
    Serial.printf("\nTry swapping SDIO<->SCLK: change PIN_SDIO=%d PIN_SCLK=%d\n", PIN_SCLK, PIN_SDIO);
    Serial.printf("Or try swapping CSB<->FCSB: change PIN_CSB=%d PIN_FCSB=%d\n", PIN_FCSB, PIN_CSB);
  } else if (chip_mode == 0x02) {
    Serial.println("PASS: CMT2300A is alive and in STANDBY mode!");
    Serial.println("SPI communication is working correctly.");
  } else {
    Serial.println("PARTIAL: Got some data but chip not in expected mode.");
    Serial.println("Check wiring and try again.");
  }

  Serial.println("\n=== Test Complete ===");
}

void loop() {
  // Nothing — one-shot test
  delay(10000);
}
