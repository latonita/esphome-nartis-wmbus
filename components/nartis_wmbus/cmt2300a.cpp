#include "cmt2300a.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace nartis_wmbus {

static const char *const TAG = "nartis_wmbus.cmt2300a";

// ============================================================================
// Bit-banged 3-wire SPI: MSB first
// Protocol: CSB low -> 8-bit addr (bit7=0 for write, 1 for read) -> 8-bit data -> CSB high
// ============================================================================

void CMT2300A::spi_write_byte_(uint8_t byte) {
  for (int8_t i = 7; i >= 0; i--) {
    pin_sclk_->digital_write(false);
    pin_sdio_->digital_write((byte >> i) & 1);
    delayMicroseconds(1);
    pin_sclk_->digital_write(true);
    delayMicroseconds(1);
  }
  pin_sclk_->digital_write(false);
}

uint8_t CMT2300A::spi_read_byte_() {
  uint8_t byte = 0;
  pin_sdio_->pin_mode(gpio::FLAG_INPUT);
  for (int8_t i = 7; i >= 0; i--) {
    pin_sclk_->digital_write(false);
    delayMicroseconds(1);
    pin_sclk_->digital_write(true);
    if (pin_sdio_->digital_read())
      byte |= (1 << i);
    delayMicroseconds(1);
  }
  pin_sclk_->digital_write(false);
  pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  return byte;
}

void CMT2300A::write_reg(uint8_t addr, uint8_t value) {
  pin_csb_->digital_write(false);
  delayMicroseconds(1);
  spi_write_byte_(addr & 0x7F);  // bit7 = 0 for write (per datasheet & firmware)
  spi_write_byte_(value);
  pin_csb_->digital_write(true);
  delayMicroseconds(1);
}

uint8_t CMT2300A::read_reg(uint8_t addr) {
  pin_csb_->digital_write(false);
  delayMicroseconds(1);
  spi_write_byte_(addr | 0x80);  // bit7 = 1 for read (per datasheet & firmware)
  uint8_t value = spi_read_byte_();
  pin_csb_->digital_write(true);
  delayMicroseconds(1);
  return value;
}

void CMT2300A::write_bank(uint8_t start_reg, const uint8_t *data, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    write_reg(start_reg + i, data[i]);
  }
}

void CMT2300A::write_fifo(const uint8_t *data, uint16_t len) {
  pin_fcsb_->digital_write(false);
  delayMicroseconds(1);
  for (uint16_t i = 0; i < len; i++) {
    spi_write_byte_(data[i]);
  }
  pin_fcsb_->digital_write(true);
  delayMicroseconds(1);
}

void CMT2300A::read_fifo(uint8_t *data, uint16_t len) {
  pin_fcsb_->digital_write(false);
  delayMicroseconds(1);
  pin_sdio_->pin_mode(gpio::FLAG_INPUT);
  for (uint16_t i = 0; i < len; i++) {
    data[i] = 0;
    for (int8_t bit = 7; bit >= 0; bit--) {
      pin_sclk_->digital_write(false);
      delayMicroseconds(1);
      pin_sclk_->digital_write(true);
      if (pin_sdio_->digital_read())
        data[i] |= (1 << bit);
      delayMicroseconds(1);
    }
    pin_sclk_->digital_write(false);
  }
  pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  pin_fcsb_->digital_write(true);
  delayMicroseconds(1);
}

// ============================================================================
// Register Configuration
// ============================================================================

void CMT2300A::write_config_(const uint8_t *config, uint8_t channel) {
  uint8_t offset = 0;
  uint8_t ch = (channel < 4) ? channel : 1;
  uint8_t tmp;

  // Pre-config: firmware function 0x13FEA does these before writing banks:
  // RMW on CUS_MODE_STA: clear RSTN_IN_EN, set CFG_RETAIN
  tmp = read_reg(CMT2300A_CUS_MODE_STA);
  write_reg(CMT2300A_CUS_MODE_STA, (tmp & ~CMT2300A_MASK_RSTN_IN_EN) | CMT2300A_MASK_CFG_RETAIN);
  // RMW on CUS_EN_CTL: set ERROR_STOP_EN
  tmp = read_reg(CMT2300A_CUS_EN_CTL);
  write_reg(CMT2300A_CUS_EN_CTL, tmp | CMT2300A_MASK_ERROR_STOP_EN);

  // Write 6 register banks in order
  write_bank(CMT2300A_CMT_BANK_ADDR, &config[offset], CMT_BANK_CUS_CMT);
  offset += CMT_BANK_CUS_CMT;

  write_bank(CMT2300A_SYSTEM_BANK_ADDR, &config[offset], CMT_BANK_CUS_SYS);
  offset += CMT_BANK_CUS_SYS;

  // Use channel table for CUS_FREQ instead of default from config
  write_bank(CMT2300A_FREQUENCY_BANK_ADDR, CMT2300A_FREQ_CHANNELS[ch], CMT_BANK_CUS_FREQ);
  offset += CMT_BANK_CUS_FREQ;

  write_bank(CMT2300A_DATA_RATE_BANK_ADDR, &config[offset], CMT_BANK_CUS_DATA_RATE);
  offset += CMT_BANK_CUS_DATA_RATE;

  write_bank(CMT2300A_BASEBAND_BANK_ADDR, &config[offset], CMT_BANK_CUS_BASEBAND);
  offset += CMT_BANK_CUS_BASEBAND;

  write_bank(CMT2300A_TX_BANK_ADDR, &config[offset], CMT_BANK_CUS_TX);

  // Post-bank fixup: CUS_CMT10 — clear bits 2:0, set bit 1
  // Firmware does this at end of both TX and RX config functions
  tmp = read_reg(CMT2300A_CUS_CMT10);
  write_reg(CMT2300A_CUS_CMT10, (tmp & 0xF8) | 0x02);

  // Apply post-config fixups (firmware function 0x13666)
  apply_fixups_();
}

void CMT2300A::apply_fixups_() {
  // Post-config register fixups — firmware function 0x13666.
  // Control Bank 1 (0x60-0x6A) is never bulk-written; always individual RMW.
  // SDK names from cmt2300a_defs.h. See decompiled/cmt2300a_fixups.c.

  uint8_t tmp;

  // 1: CUS_IO_SEL — GPIO pin mode: GPIO2=INT1, GPIO3=INT2, GPIO1=DOUT/DIN
  write_reg(CMT2300A_CUS_IO_SEL, CMT2300A_GPIO3_SEL_INT2 | CMT2300A_GPIO2_SEL_INT1);

  // 2: CUS_INT2_CTL — preserve top 3 bits, set INT2_SEL = RX_FIFO_TH
  tmp = read_reg(CMT2300A_CUS_INT2_CTL);
  write_reg(CMT2300A_CUS_INT2_CTL, (tmp & 0xE0) | CMT2300A_INT_SEL_RX_FIFO_TH);

  // 3: CUS_INT_EN = TX_DONE + PREAM_OK + SYNC_OK + PKT_DONE
  write_reg(CMT2300A_CUS_INT_EN,
            CMT2300A_MASK_TX_DONE_EN | CMT2300A_MASK_PREAM_OK_EN |
            CMT2300A_MASK_SYNC_OK_EN | CMT2300A_MASK_PKT_DONE_EN);

  // 4: CUS_SYS2 — clear bits 7:5 (disable LFOSC calibration timers)
  tmp = read_reg(CMT2300A_CUS_SYS2);
  write_reg(CMT2300A_CUS_SYS2, tmp & ~(CMT2300A_MASK_LFOSC_RECAL_EN |
            CMT2300A_MASK_LFOSC_CAL1_EN | CMT2300A_MASK_LFOSC_CAL2_EN));

  // 5: CUS_FIFO_CTL — set FIFO_MERGE_EN (merge TX+RX → 64 bytes)
  tmp = read_reg(CMT2300A_CUS_FIFO_CTL);
  write_reg(CMT2300A_CUS_FIFO_CTL, tmp | CMT2300A_MASK_FIFO_MERGE_EN);

  // 6: CUS_PKT29 — preserve FIFO_AUTO_RES_EN, set FIFO threshold to 15
  tmp = read_reg(CMT2300A_CUS_PKT29);
  write_reg(CMT2300A_CUS_PKT29, (tmp & CMT2300A_MASK_FIFO_AUTO_RES_EN) | 0x0F);

  // 7: CUS_SYS11 — preserve top 3, set low 5 to 0x12 (RSSI config)
  tmp = read_reg(CMT2300A_CUS_SYS11);
  write_reg(CMT2300A_CUS_SYS11, (tmp & 0xE0) | 0x12);

  // 8a: TX power level=0 — CUS_CMT4 + CUS_TX8/TX9 (PA config)
  write_reg(CMT2300A_CUS_CMT4, 0x1C);
  write_reg(CMT2300A_CUS_TX8, 0x10);
  write_reg(CMT2300A_CUS_TX9, 0x02);

  // 8b: Freq config=0 — CDR/AFC tuning (PKT10-PKT13 in Baseband Bank)
  write_reg(CMT2300A_CUS_PKT10, 0x8D);
  write_reg(CMT2300A_CUS_PKT11, 0xF6);
  write_reg(CMT2300A_CUS_PKT12, 0x55);
  write_reg(CMT2300A_CUS_PKT13, 0x55);

  // 8d: CUS_INT1_CTL — default INT1 source (overridden by start_rx for ISR mode)
  set_int1_source_(CMT2300A_INT_SEL_TX_FIFO_NMTY);

  // Firmware ends fixups by going to SLEEP
  // Caller transitions to needed mode afterward
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_SLEEP);
}

void CMT2300A::set_int1_source_(uint8_t source) {
  write_reg(CMT2300A_CUS_INT1_CTL, source);
}

// ============================================================================
// Mode Control
// ============================================================================

bool CMT2300A::wait_for_mode_(uint8_t expected_mode, uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t status = read_reg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
    if (status == expected_mode)
      return true;
    delayMicroseconds(100);
  }
  ESP_LOGW(TAG, "Timeout waiting for mode 0x%02X", expected_mode);
  return false;
}

void CMT2300A::clear_interrupts_() {
  // Clear all interrupt flags by writing clear bits
  // CUS_INT_CLR1: clear TX_DONE, SL_TMO, RX_TMO
  write_reg(CMT2300A_CUS_INT_CLR1,
            CMT2300A_MASK_TX_DONE_CLR | CMT2300A_MASK_SL_TMO_CLR | CMT2300A_MASK_RX_TMO_CLR);
  // CUS_INT_CLR2: clear LBD, PREAM_OK, SYNC_OK, NODE_OK, CRC_OK, PKT_DONE
  write_reg(CMT2300A_CUS_INT_CLR2,
            CMT2300A_MASK_LBD_CLR | CMT2300A_MASK_PREAM_OK_CLR | CMT2300A_MASK_SYNC_OK_CLR |
            CMT2300A_MASK_NODE_OK_CLR | CMT2300A_MASK_CRC_OK_CLR | CMT2300A_MASK_PKT_DONE_CLR);
}

void CMT2300A::clear_fifo_() {
  // Per firmware at 0x13E16/0x13E2C: write to CUS_FIFO_CLR
  // Write both to clear entire FIFO
  write_reg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_TX);
  write_reg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_RX);
}

bool CMT2300A::go_standby() {
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  return wait_for_mode_(CMT2300A_STA_STBY);
}

// ============================================================================
// ISR-driven RX (ESP32 only)
// ============================================================================

#ifdef USE_ESP32

void IRAM_ATTR CMT2300A::gpio1_isr_(CMT2300A *arg) {
  // ISR context — only send notification, no SPI here (bit-banged GPIO not ISR-safe)
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(arg->rx_task_handle_, &woken);
  portYIELD_FROM_ISR(woken);
}

void CMT2300A::receiver_task_(void *arg) {
  CMT2300A *radio = (CMT2300A *) arg;

  while (true) {
    // Block until GPIO1 ISR fires or 60s timeout (safety watchdog)
    uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000));

    if (!radio->rx_active_)
      continue;  // not in RX mode, ignore stale interrupts

    if (notification == 0) {
      // 60s timeout with no packet — restart RX to recover from stuck state
      ESP_LOGD(TAG, "RX task watchdog — restarting RX");
      radio->clear_interrupts_();
      radio->clear_fifo_();
      radio->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
      continue;
    }

    // ISR fired — read interrupt flags via SPI (safe in task context)
    uint8_t flags = radio->read_reg(CMT2300A_CUS_INT_FLAG);

    if (flags & CMT2300A_MASK_PKT_OK_FLG) {
      uint8_t pkt_len = radio->read_reg(CMT2300A_CUS_PKT7);
      if (pkt_len > 0 && pkt_len <= CMT_MAX_PKT_SIZE) {
        RxPacket pkt;
        pkt.len = pkt_len;
        radio->read_fifo(pkt.data, pkt_len);
        ESP_LOGD(TAG, "RX task: %d bytes", pkt_len);

        // Non-blocking push to queue — drop if full
        if (xQueueSend(radio->rx_queue_, &pkt, 0) != pdTRUE) {
          ESP_LOGW(TAG, "RX queue full, packet dropped");
        }
      } else {
        ESP_LOGW(TAG, "RX task: bad length %d", pkt_len);
      }

      // Clear and re-enter RX for next packet
      radio->clear_interrupts_();
      radio->clear_fifo_();
      radio->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);

    } else if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
      ESP_LOGW(TAG, "RX task: CRC error");
      radio->clear_interrupts_();
      radio->clear_fifo_();
      radio->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);

    } else {
      // Spurious interrupt — clear and continue
      radio->clear_interrupts_();
    }
  }
}

bool CMT2300A::init_isr_() {
  if (pin_gpio1_ == nullptr)
    return false;

  // Create packet queue (depth 3)
  rx_queue_ = xQueueCreate(3, sizeof(RxPacket));
  if (rx_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    return false;
  }

  // Create receiver task (3KB stack, priority 2 — above idle, below WiFi)
  BaseType_t ret = xTaskCreatePinnedToCore(
      receiver_task_, "cmt_rx", 3 * 1024, this, 2, &rx_task_handle_, 1);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task");
    vQueueDelete(rx_queue_);
    rx_queue_ = nullptr;
    return false;
  }

  // Attach GPIO1 interrupt (RISING edge = INT1 asserted)
  pin_gpio1_->attach_interrupt(gpio1_isr_, this, gpio::INTERRUPT_RISING_EDGE);

  isr_enabled_ = true;
  ESP_LOGD(TAG, "ISR-driven RX enabled on GPIO1");
  return true;
}

void CMT2300A::deinit_isr_() {
  if (!isr_enabled_)
    return;

  pin_gpio1_->detach_interrupt();

  if (rx_task_handle_ != nullptr) {
    vTaskDelete(rx_task_handle_);
    rx_task_handle_ = nullptr;
  }
  if (rx_queue_ != nullptr) {
    vQueueDelete(rx_queue_);
    rx_queue_ = nullptr;
  }

  isr_enabled_ = false;
}

#else  // !USE_ESP32

bool CMT2300A::init_isr_() { return false; }
void CMT2300A::deinit_isr_() {}

#endif  // USE_ESP32

// ============================================================================
// Initialization
// ============================================================================

bool CMT2300A::init(uint8_t channel) {
  ESP_LOGD(TAG, "Initializing CMT2300A, channel %d (%.3f MHz)", channel, CMT2300A_FREQ_MHZ[channel < 4 ? channel : 1]);

  // Setup GPIO pins
  pin_sdio_->setup();
  pin_sclk_->setup();
  pin_csb_->setup();
  pin_fcsb_->setup();

  pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  pin_sclk_->pin_mode(gpio::FLAG_OUTPUT);
  pin_csb_->pin_mode(gpio::FLAG_OUTPUT);
  pin_fcsb_->pin_mode(gpio::FLAG_OUTPUT);

  // Set idle states
  pin_csb_->digital_write(true);
  pin_fcsb_->digital_write(true);
  pin_sclk_->digital_write(false);

  // Allow SPI interface to sync (firmware sends 10 clock pulses; delay is equivalent)
  delay(1);

  if (pin_gpio1_ != nullptr) {
    pin_gpio1_->setup();
    pin_gpio1_->pin_mode(gpio::FLAG_INPUT);
  }

  // Soft reset: firmware writes 0xFF to CUS_SOFTRST, then go_standby
  // See firmware function 0x13C7C
  write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(10);
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  delay(10);

  if (!wait_for_mode_(CMT2300A_STA_STBY, 100)) {
    ESP_LOGE(TAG, "CMT2300A failed to enter standby mode");
    return false;
  }

  // Write RX config as default idle config
  write_config_(CMT2300A_RX_CONFIG, channel);

  // write_config_ ends in SLEEP, return to STBY
  if (!go_standby()) {
    ESP_LOGE(TAG, "CMT2300A failed to return to standby after config");
    return false;
  }

  clear_interrupts_();
  clear_fifo_();

  // Verify chip: read back a known register
  uint8_t check = read_reg(CMT2300A_CUS_CMT1);
  if (check == 0xFF || check == 0x00) {
    ESP_LOGE(TAG, "CMT2300A not responding (read 0x%02X from reg 0x00)", check);
    return false;
  }

  // Try to initialize ISR-driven RX (falls back to polling if GPIO1 not available)
  if (!init_isr_()) {
    ESP_LOGD(TAG, "GPIO1 not available — using polling RX");
  }

  ESP_LOGD(TAG, "CMT2300A initialized OK (reg0=0x%02X, isr=%s)", check, isr_enabled_ ? "yes" : "no");
  return true;
}

// ============================================================================
// TX / RX Mode Switching
// ============================================================================

bool CMT2300A::switch_tx(uint8_t channel) {
  if (!go_standby())
    return false;

  write_config_(CMT2300A_TX_CONFIG, channel);

  // write_config_ ends in SLEEP (per firmware). Return to STBY for FIFO access.
  if (!go_standby())
    return false;

  clear_interrupts_();
  clear_fifo_();
  return true;
}

bool CMT2300A::switch_rx(uint8_t channel) {
  if (!go_standby())
    return false;

  write_config_(CMT2300A_RX_CONFIG, channel);

  // write_config_ ends in SLEEP (per firmware). Return to STBY for FIFO access.
  if (!go_standby())
    return false;

  clear_interrupts_();
  clear_fifo_();
  return true;
}

// ============================================================================
// Packet TX (blocking — OK, TX is fast: ~100ms max for W-MBus frame)
// ============================================================================

bool CMT2300A::send_packet(const uint8_t *data, uint16_t len, uint8_t channel) {
  ESP_LOGD(TAG, "TX %d bytes on ch%d", len, channel);

  if (!switch_tx(channel))
    return false;

  // Write data to FIFO
  write_fifo(data, len);

  // Set packet length (CUS_PKT7 stores payload length)
  write_reg(CMT2300A_CUS_PKT7, len);

  // Go TX
  clear_interrupts_();
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_TX);

  // Wait for TX done (poll status, ~100ms max for typical W-MBus frame)
  uint32_t start = millis();
  while (millis() - start < 200) {
    // TX_DONE flag is in CUS_INT_CLR1 (0x6A) bit 3 (read-only)
    uint8_t clr1 = read_reg(CMT2300A_CUS_INT_CLR1);
    if (clr1 & CMT2300A_MASK_TX_DONE_FLG) {
      ESP_LOGD(TAG, "TX complete");
      go_standby();
      return true;
    }
    // Also check mode status — if back to standby, TX is done
    uint8_t mode = read_reg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
    if (mode == CMT2300A_STA_STBY) {
      ESP_LOGD(TAG, "TX complete (mode=STBY)");
      return true;
    }
    delayMicroseconds(500);
  }

  ESP_LOGW(TAG, "TX timeout");
  go_standby();
  return false;
}

// ============================================================================
// Blocking RX (used by sniffer mode only)
// ============================================================================

uint16_t CMT2300A::receive_packet(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms, uint8_t channel) {
  if (!switch_rx(channel))
    return 0;

  // Enter RX mode
  clear_interrupts_();
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);

  // Poll for packet received
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t flags = read_reg(CMT2300A_CUS_INT_FLAG);
    if (flags & CMT2300A_MASK_PKT_OK_FLG) {
      // Read packet length from FIFO status
      uint8_t pkt_len = read_reg(CMT2300A_CUS_PKT7);  // RX payload length register
      if (pkt_len == 0 || pkt_len > max_len) {
        ESP_LOGW(TAG, "RX bad length: %d", pkt_len);
        go_standby();
        return 0;
      }

      read_fifo(buf, pkt_len);
      ESP_LOGD(TAG, "RX %d bytes", pkt_len);
      go_standby();
      return pkt_len;
    }

    // Check for packet error
    if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
      ESP_LOGW(TAG, "RX packet error");
      clear_interrupts_();
      clear_fifo_();
      // Re-enter RX
      write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
    }

    delay(1);
  }

  go_standby();
  return 0;
}

// ============================================================================
// Non-blocking RX API
// ============================================================================

bool CMT2300A::start_rx(uint8_t channel) {
  if (!switch_rx(channel))
    return false;

  // Configure INT1 for PKT_DONE (fires on both OK and error packets)
  if (isr_enabled_) {
    set_int1_source_(CMT2300A_INT_SEL_PKT_DONE);
  }

  clear_interrupts_();
  write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
  rx_active_ = true;

#ifdef USE_ESP32
  // Drain any stale packets from queue
  if (isr_enabled_ && rx_queue_ != nullptr) {
    RxPacket discard;
    while (xQueueReceive(rx_queue_, &discard, 0) == pdTRUE) {}
  }
#endif

  return true;
}

int16_t CMT2300A::check_rx(uint8_t *buf, uint16_t max_len) {
  if (isr_enabled_) {
    return check_rx_isr_(buf, max_len);
  }
  return check_rx_polling_(buf, max_len);
}

void CMT2300A::stop_rx() {
  rx_active_ = false;

  // Restore INT1 source to default (TX_FIFO_NMTY)
  if (isr_enabled_) {
    set_int1_source_(CMT2300A_INT_SEL_TX_FIFO_NMTY);
  }

  go_standby();
}

// Polling fallback: read interrupt flags via SPI each call (~50μs)
int16_t CMT2300A::check_rx_polling_(uint8_t *buf, uint16_t max_len) {
  uint8_t flags = read_reg(CMT2300A_CUS_INT_FLAG);

  if (flags & CMT2300A_MASK_PKT_OK_FLG) {
    uint8_t pkt_len = read_reg(CMT2300A_CUS_PKT7);
    if (pkt_len == 0 || pkt_len > max_len) {
      ESP_LOGW(TAG, "RX bad length: %d", pkt_len);
      return -1;
    }
    read_fifo(buf, pkt_len);
    ESP_LOGD(TAG, "RX %d bytes", pkt_len);
    return pkt_len;
  }

  if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
    ESP_LOGW(TAG, "RX packet error");
    clear_interrupts_();
    clear_fifo_();
    write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);  // re-enter RX
  }

  return 0;  // nothing yet
}

// ISR-driven: instant check of FreeRTOS queue (no SPI)
int16_t CMT2300A::check_rx_isr_(uint8_t *buf, uint16_t max_len) {
#ifdef USE_ESP32
  if (rx_queue_ == nullptr)
    return 0;

  RxPacket pkt;
  if (xQueueReceive(rx_queue_, &pkt, 0) != pdTRUE)
    return 0;  // nothing yet — instant return

  if (pkt.len > max_len) {
    ESP_LOGW(TAG, "RX packet too large for buffer: %d > %d", pkt.len, max_len);
    return -1;
  }

  memcpy(buf, pkt.data, pkt.len);
  return pkt.len;
#else
  return check_rx_polling_(buf, max_len);
#endif
}

}  // namespace nartis_wmbus
}  // namespace esphome
