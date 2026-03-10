#include "cmt2300a.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome::nartis_wmbus {

static const char *const TAG = "nartis_wmbus.cmt2300a";

// ============================================================================
// Bit-banged 3-wire SPI: MSB first
// Protocol: CSB low -> 8-bit addr (bit7=0 for write, 1 for read) -> 8-bit data -> CSB high
// ============================================================================

void CMT2300A::spi_write_byte_(uint8_t byte) {
  for (int8_t i = 7; i >= 0; i--) {
    this->pin_sclk_->digital_write(false);
    this->pin_sdio_->digital_write((byte >> i) & 1);
    delayMicroseconds(1);
    this->pin_sclk_->digital_write(true);
    delayMicroseconds(1);
  }
  this->pin_sclk_->digital_write(false);
}

uint8_t CMT2300A::spi_read_byte_() {
  uint8_t byte = 0;
  this->pin_sdio_->pin_mode(gpio::FLAG_INPUT);
  for (int8_t i = 7; i >= 0; i--) {
    this->pin_sclk_->digital_write(false);
    delayMicroseconds(1);
    this->pin_sclk_->digital_write(true);
    if (this->pin_sdio_->digital_read())
      byte |= (1 << i);
    delayMicroseconds(1);
  }
  this->pin_sclk_->digital_write(false);
  this->pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  return byte;
}

void CMT2300A::write_reg(uint8_t addr, uint8_t value) {
  this->pin_csb_->digital_write(false);
  delayMicroseconds(1);
  this->spi_write_byte_(addr & 0x7F);  // bit7 = 0 for write (per datasheet & firmware)
  this->spi_write_byte_(value);
  this->pin_csb_->digital_write(true);
  delayMicroseconds(1);
}

uint8_t CMT2300A::read_reg(uint8_t addr) {
  this->pin_csb_->digital_write(false);
  delayMicroseconds(1);
  this->spi_write_byte_(addr | 0x80);  // bit7 = 1 for read (per datasheet & firmware)
  uint8_t value = this->spi_read_byte_();
  this->pin_csb_->digital_write(true);
  delayMicroseconds(1);
  return value;
}

void CMT2300A::write_bank(uint8_t start_reg, const uint8_t *data, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    this->write_reg(start_reg + i, data[i]);
  }
}

void CMT2300A::write_fifo(const uint8_t *data, uint16_t len) {
  this->pin_fcsb_->digital_write(false);
  delayMicroseconds(1);
  for (uint16_t i = 0; i < len; i++) {
    this->spi_write_byte_(data[i]);
  }
  this->pin_fcsb_->digital_write(true);
  delayMicroseconds(1);
}

void CMT2300A::read_fifo(uint8_t *data, uint16_t len) {
  this->pin_fcsb_->digital_write(false);
  delayMicroseconds(1);
  this->pin_sdio_->pin_mode(gpio::FLAG_INPUT);
  for (uint16_t i = 0; i < len; i++) {
    data[i] = 0;
    for (int8_t bit = 7; bit >= 0; bit--) {
      this->pin_sclk_->digital_write(false);
      delayMicroseconds(1);
      this->pin_sclk_->digital_write(true);
      if (this->pin_sdio_->digital_read())
        data[i] |= (1 << bit);
      delayMicroseconds(1);
    }
    this->pin_sclk_->digital_write(false);
  }
  this->pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_fcsb_->digital_write(true);
  delayMicroseconds(1);
}

// ============================================================================
// Register Configuration
// ============================================================================

void CMT2300A::write_config_(const uint8_t *config, uint8_t channel) {
  uint8_t offset = 0;
  uint8_t ch = (channel < 4) ? channel : 1;
  uint8_t tmp;

  ESP_LOGV(TAG, "write_config_: ch=%d (effective=%d), config=%s", channel, ch,
           config == CMT2300A_TX_CONFIG ? "TX" : config == CMT2300A_RX_CONFIG ? "RX" : "unknown");

  // Pre-config: firmware function 0x13FEA does these before writing banks:
  // RMW on CUS_MODE_STA: clear RSTN_IN_EN, set CFG_RETAIN
  tmp = this->read_reg(CMT2300A_CUS_MODE_STA);
  ESP_LOGVV(TAG, "write_config_: pre-config CUS_MODE_STA=0x%02X -> 0x%02X", tmp,
            (tmp & ~CMT2300A_MASK_RSTN_IN_EN) | CMT2300A_MASK_CFG_RETAIN);
  this->write_reg(CMT2300A_CUS_MODE_STA, (tmp & ~CMT2300A_MASK_RSTN_IN_EN) | CMT2300A_MASK_CFG_RETAIN);
  // RMW on CUS_EN_CTL: set ERROR_STOP_EN
  tmp = this->read_reg(CMT2300A_CUS_EN_CTL);
  ESP_LOGVV(TAG, "write_config_: pre-config CUS_EN_CTL=0x%02X -> 0x%02X", tmp, tmp | CMT2300A_MASK_ERROR_STOP_EN);
  this->write_reg(CMT2300A_CUS_EN_CTL, tmp | CMT2300A_MASK_ERROR_STOP_EN);

  // Write 6 register banks in order
  ESP_LOGVV(TAG, "write_config_: bank CMT (0x%02X, %d regs)", CMT2300A_CMT_BANK_ADDR, CMT_BANK_CUS_CMT);
  this->write_bank(CMT2300A_CMT_BANK_ADDR, &config[offset], CMT_BANK_CUS_CMT);
  offset += CMT_BANK_CUS_CMT;

  ESP_LOGVV(TAG, "write_config_: bank SYS (0x%02X, %d regs)", CMT2300A_SYSTEM_BANK_ADDR, CMT_BANK_CUS_SYS);
  this->write_bank(CMT2300A_SYSTEM_BANK_ADDR, &config[offset], CMT_BANK_CUS_SYS);
  offset += CMT_BANK_CUS_SYS;

  // Use channel table for CUS_FREQ instead of default from config
  ESP_LOGVV(TAG, "write_config_: bank FREQ (0x%02X, %d regs) — channel %d = %.3f MHz",
            CMT2300A_FREQUENCY_BANK_ADDR, CMT_BANK_CUS_FREQ, ch, CMT2300A_FREQ_MHZ[ch]);
  this->write_bank(CMT2300A_FREQUENCY_BANK_ADDR, CMT2300A_FREQ_CHANNELS[ch], CMT_BANK_CUS_FREQ);
  offset += CMT_BANK_CUS_FREQ;

  ESP_LOGVV(TAG, "write_config_: bank DATA_RATE (0x%02X, %d regs)", CMT2300A_DATA_RATE_BANK_ADDR, CMT_BANK_CUS_DATA_RATE);
  this->write_bank(CMT2300A_DATA_RATE_BANK_ADDR, &config[offset], CMT_BANK_CUS_DATA_RATE);
  offset += CMT_BANK_CUS_DATA_RATE;

  ESP_LOGVV(TAG, "write_config_: bank BASEBAND (0x%02X, %d regs)", CMT2300A_BASEBAND_BANK_ADDR, CMT_BANK_CUS_BASEBAND);
  this->write_bank(CMT2300A_BASEBAND_BANK_ADDR, &config[offset], CMT_BANK_CUS_BASEBAND);
  offset += CMT_BANK_CUS_BASEBAND;

  ESP_LOGVV(TAG, "write_config_: bank TX (0x%02X, %d regs)", CMT2300A_TX_BANK_ADDR, CMT_BANK_CUS_TX);
  this->write_bank(CMT2300A_TX_BANK_ADDR, &config[offset], CMT_BANK_CUS_TX);

  // Post-bank fixup: CUS_CMT10 — clear bits 2:0, set bit 1
  // Firmware does this at end of both TX and RX config functions
  tmp = this->read_reg(CMT2300A_CUS_CMT10);
  ESP_LOGVV(TAG, "write_config_: CUS_CMT10 fixup 0x%02X -> 0x%02X", tmp, (tmp & 0xF8) | 0x02);
  this->write_reg(CMT2300A_CUS_CMT10, (tmp & 0xF8) | 0x02);

  // Apply post-config fixups (firmware function 0x13666)
  bool is_tx = (config == CMT2300A_TX_CONFIG);
  ESP_LOGVV(TAG, "write_config_: applying post-config fixups (mode=%s)", is_tx ? "TX" : "RX");
  this->apply_fixups_(is_tx);
  ESP_LOGV(TAG, "write_config_: done");
}

void CMT2300A::apply_fixups_(bool is_tx) {
  // Post-config register fixups — firmware function 0x13666.
  // Control Bank 1 (0x60-0x6A) is never bulk-written; always individual RMW.
  // SDK names from cmt2300a_defs.h. See decompiled/cmt2300a_fixups.c.

  uint8_t tmp;

  // 1: CUS_IO_SEL — GPIO pin routing
  // TX: GPIO3=INT2 (unused), GPIO2=INT1, GPIO1=DOUT/DIN
  // RX: GPIO3=DOUT (data bits), GPIO2=INT1 (SYNC_OK), GPIO1=DOUT/DIN
  uint8_t io_sel_val;
  if (is_tx) {
    io_sel_val = CMT2300A_GPIO3_SEL_INT2 | CMT2300A_GPIO2_SEL_INT1;
    ESP_LOGVV(TAG, "fixup 1: CUS_IO_SEL = 0x%02X (TX: GPIO3=INT2, GPIO2=INT1)", io_sel_val);
  } else {
    io_sel_val = CMT2300A_GPIO3_SEL_DOUT | CMT2300A_GPIO2_SEL_INT1;
    ESP_LOGVV(TAG, "fixup 1: CUS_IO_SEL = 0x%02X (RX: GPIO3=DOUT, GPIO2=INT1)", io_sel_val);
  }
  this->write_reg(CMT2300A_CUS_IO_SEL, io_sel_val);

  // 2: CUS_INT2_CTL — preserve top 3 bits, set INT2_SEL = RX_FIFO_TH
  tmp = this->read_reg(CMT2300A_CUS_INT2_CTL);
  ESP_LOGVV(TAG, "fixup 2: CUS_INT2_CTL 0x%02X -> 0x%02X", tmp, (tmp & 0xE0) | CMT2300A_INT_SEL_RX_FIFO_TH);
  this->write_reg(CMT2300A_CUS_INT2_CTL, (tmp & 0xE0) | CMT2300A_INT_SEL_RX_FIFO_TH);

  // 3: CUS_INT_EN = TX_DONE + PREAM_OK + SYNC_OK + PKT_DONE
  uint8_t int_en_val = CMT2300A_MASK_TX_DONE_EN | CMT2300A_MASK_PREAM_OK_EN | CMT2300A_MASK_SYNC_OK_EN |
                       CMT2300A_MASK_PKT_DONE_EN;
  ESP_LOGVV(TAG, "fixup 3: CUS_INT_EN = 0x%02X", int_en_val);
  this->write_reg(CMT2300A_CUS_INT_EN, int_en_val);

  // 4: CUS_SYS2 — clear bits 7:5 (disable LFOSC calibration timers)
  tmp = this->read_reg(CMT2300A_CUS_SYS2);
  ESP_LOGVV(TAG, "fixup 4: CUS_SYS2 0x%02X -> 0x%02X", tmp,
            tmp & ~(CMT2300A_MASK_LFOSC_RECAL_EN | CMT2300A_MASK_LFOSC_CAL1_EN | CMT2300A_MASK_LFOSC_CAL2_EN));
  this->write_reg(CMT2300A_CUS_SYS2,
                  tmp & ~(CMT2300A_MASK_LFOSC_RECAL_EN | CMT2300A_MASK_LFOSC_CAL1_EN | CMT2300A_MASK_LFOSC_CAL2_EN));

  // 5: CUS_FIFO_CTL — set FIFO_MERGE_EN (merge TX+RX → 64 bytes)
  tmp = this->read_reg(CMT2300A_CUS_FIFO_CTL);
  ESP_LOGVV(TAG, "fixup 5: CUS_FIFO_CTL 0x%02X -> 0x%02X", tmp, tmp | CMT2300A_MASK_FIFO_MERGE_EN);
  this->write_reg(CMT2300A_CUS_FIFO_CTL, tmp | CMT2300A_MASK_FIFO_MERGE_EN);

  // 6: CUS_PKT29 — preserve FIFO_AUTO_RES_EN, set FIFO threshold to 15
  tmp = this->read_reg(CMT2300A_CUS_PKT29);
  ESP_LOGVV(TAG, "fixup 6: CUS_PKT29 0x%02X -> 0x%02X", tmp, (tmp & CMT2300A_MASK_FIFO_AUTO_RES_EN) | 0x0F);
  this->write_reg(CMT2300A_CUS_PKT29, (tmp & CMT2300A_MASK_FIFO_AUTO_RES_EN) | 0x0F);

  // 7: CUS_SYS11 — preserve top 3, set low 5 to 0x12 (RSSI config)
  tmp = this->read_reg(CMT2300A_CUS_SYS11);
  ESP_LOGVV(TAG, "fixup 7: CUS_SYS11 0x%02X -> 0x%02X", tmp, (tmp & 0xE0) | 0x12);
  this->write_reg(CMT2300A_CUS_SYS11, (tmp & 0xE0) | 0x12);

  // 8a: TX power level=0 — CUS_CMT4 + CUS_TX8/TX9 (PA config)
  ESP_LOGVV(TAG, "fixup 8a: TX power — CMT4=0x1C TX8=0x10 TX9=0x02");
  this->write_reg(CMT2300A_CUS_CMT4, 0x1C);
  this->write_reg(CMT2300A_CUS_TX8, 0x10);
  this->write_reg(CMT2300A_CUS_TX9, 0x02);

  // 8b: Freq config=0 — CDR/AFC tuning + sync word (PKT10-PKT13 in Baseband Bank)
  // PKT12/PKT13 are sync word registers — always write correct values for mode.
  // Firmware always writes these (set_freq_config at 0x1334e), values are same for both modes
  // except sync word which differs: TX=0x55,0x55  RX=0xD4,0x2D
  this->write_reg(CMT2300A_CUS_PKT10, 0x8D);
  this->write_reg(CMT2300A_CUS_PKT11, 0xF6);
  if (is_tx) {
    ESP_LOGVV(TAG, "fixup 8b: CDR/AFC — PKT10=0x8D PKT11=0xF6 PKT12=0x55 PKT13=0x55 (TX sync)");
    this->write_reg(CMT2300A_CUS_PKT12, 0x55);
    this->write_reg(CMT2300A_CUS_PKT13, 0x55);
  } else {
    ESP_LOGVV(TAG, "fixup 8b: CDR/AFC — PKT10=0x8D PKT11=0xF6 PKT12=0xD4 PKT13=0x2D (RX sync)");
    this->write_reg(CMT2300A_CUS_PKT12, 0xD4);
    this->write_reg(CMT2300A_CUS_PKT13, 0x2D);
  }

  // 8d: CUS_INT1_CTL — default INT1 source (overridden by start_rx for ISR mode)
  ESP_LOGVV(TAG, "fixup 8d: CUS_INT1_CTL = 0x%02X (TX_FIFO_NMTY)", CMT2300A_INT_SEL_TX_FIFO_NMTY);
  this->set_int1_source_(CMT2300A_INT_SEL_TX_FIFO_NMTY);

  // Firmware ends fixups by going to SLEEP
  // Caller transitions to needed mode afterward
  ESP_LOGVV(TAG, "fixups done, going to SLEEP");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_SLEEP);
}

void CMT2300A::set_int1_source_(uint8_t source) { this->write_reg(CMT2300A_CUS_INT1_CTL, source); }

// ============================================================================
// Mode Control
// ============================================================================

bool CMT2300A::wait_for_mode_(uint8_t expected_mode, uint32_t timeout_ms) {
  uint32_t start = millis();
  ESP_LOGVV(TAG, "wait_for_mode_: expecting 0x%02X, timeout %ums", expected_mode, timeout_ms);
  while (millis() - start < timeout_ms) {
    uint8_t raw = this->read_reg(CMT2300A_CUS_MODE_STA);
    uint8_t status = raw & CMT2300A_MASK_CHIP_MODE_STA;
    if (status == expected_mode) {
      ESP_LOGVV(TAG, "wait_for_mode_: reached 0x%02X in %ums (raw MODE_STA=0x%02X)", expected_mode,
                millis() - start, raw);
      return true;
    }
    delayMicroseconds(100);
  }
  uint8_t final_raw = this->read_reg(CMT2300A_CUS_MODE_STA);
  ESP_LOGW(TAG, "Timeout waiting for mode 0x%02X after %ums (current MODE_STA=0x%02X, mode=0x%02X)",
           expected_mode, timeout_ms, final_raw, final_raw & CMT2300A_MASK_CHIP_MODE_STA);
  return false;
}

void CMT2300A::clear_interrupts_() {
  // Clear all interrupt flags by writing clear bits
  // CUS_INT_CLR1: clear TX_DONE, SL_TMO, RX_TMO
  this->write_reg(CMT2300A_CUS_INT_CLR1,
                  CMT2300A_MASK_TX_DONE_CLR | CMT2300A_MASK_SL_TMO_CLR | CMT2300A_MASK_RX_TMO_CLR);
  // CUS_INT_CLR2: clear LBD, PREAM_OK, SYNC_OK, NODE_OK, CRC_OK, PKT_DONE
  this->write_reg(CMT2300A_CUS_INT_CLR2, CMT2300A_MASK_LBD_CLR | CMT2300A_MASK_PREAM_OK_CLR |
                                             CMT2300A_MASK_SYNC_OK_CLR | CMT2300A_MASK_NODE_OK_CLR |
                                             CMT2300A_MASK_CRC_OK_CLR | CMT2300A_MASK_PKT_DONE_CLR);
}

void CMT2300A::clear_fifo_() {
  // Per firmware at 0x13E16/0x13E2C: write to CUS_FIFO_CLR
  // Write both to clear entire FIFO
  this->write_reg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_TX);
  this->write_reg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_RX);
}

bool CMT2300A::go_standby() {
  ESP_LOGVV(TAG, "go_standby: sending GO_STBY");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  bool ok = this->wait_for_mode_(CMT2300A_STA_STBY);
  if (!ok) {
    ESP_LOGW(TAG, "go_standby: FAILED to reach STBY");
  }
  return ok;
}

// ============================================================================
// Direct Mode RX (ESP32 only)
// ============================================================================
// CMT2300A demodulates RF and outputs raw bits on GPIO3 (DOUT).
// GPIO2 (INT1) signals SYNC_OK when sync word 0xD42D is detected.
// After SYNC_OK, we sample DOUT at 2400 Hz using esp_timer to reconstruct bytes.
// L-field (first byte) determines total frame length including CRC blocks.
// ============================================================================

#ifdef USE_ESP32

void IRAM_ATTR CMT2300A::sync_ok_isr_(void *arg) {
  CMT2300A *radio = static_cast<CMT2300A *>(arg);
  DirectRxState &drx = radio->drx_;

  // Ignore if already sampling a frame
  if (drx.active)
    return;

  // Reset state for new frame
  drx.byte_count = 0;
  drx.bit_count = 0;
  drx.current_byte = 0;
  drx.expected_air_len = 0;
  drx.frame_ready = false;
  drx.total_bits = 0;
  drx.active = true;

  // Start sampling timer — half-bit delay to center on first data bit
  esp_timer_start_once(radio->sample_timer_, DIRECT_RX_HALF_BIT_US);
}

void IRAM_ATTR CMT2300A::sample_timer_cb_(void *arg) {
  CMT2300A *radio = static_cast<CMT2300A *>(arg);
  DirectRxState &drx = radio->drx_;

  if (!drx.active)
    return;

  // Read DOUT pin (direct register access for speed)
  bool bit = gpio_get_level(drx.dout_gpio_num) != 0;

  // Accumulate MSB first (per firmware PKT14 PAYLOAD_BIT_ORDER=0)
  drx.current_byte = (drx.current_byte << 1) | (bit ? 1 : 0);
  drx.bit_count++;
  drx.total_bits++;

  if (drx.bit_count == 8) {
    drx.frame_buf[drx.byte_count] = drx.current_byte;
    drx.byte_count++;
    drx.bit_count = 0;
    drx.current_byte = 0;

    // After L-field (first byte), calculate expected air frame length
    if (drx.byte_count == 1) {
      uint8_t l_field = drx.frame_buf[0];
      if (l_field < 10 || l_field > 250) {
        // Invalid L-field — noise after false sync
        drx.active = false;
        return;  // timer stops (one-shot)
      }
      drx.expected_air_len = wmbus_air_frame_len(l_field);
      if (drx.expected_air_len > DIRECT_RX_MAX_FRAME)
        drx.expected_air_len = DIRECT_RX_MAX_FRAME;
    }

    // Frame complete?
    if (drx.expected_air_len > 0 && drx.byte_count >= drx.expected_air_len) {
      drx.active = false;
      drx.frame_ready = true;
      return;  // timer stops (one-shot)
    }

    // Buffer overflow guard
    if (drx.byte_count >= DIRECT_RX_MAX_FRAME) {
      drx.active = false;
      drx.frame_ready = true;
      return;
    }
  }

  // Timeout guard
  if (drx.total_bits >= DIRECT_RX_TIMEOUT_BITS) {
    drx.active = false;
    drx.frame_ready = (drx.byte_count > 0);
    return;
  }

  // Schedule next sample (one-shot chaining for precise timing)
  esp_timer_start_once(radio->sample_timer_, DIRECT_RX_BIT_PERIOD_US);
}

bool CMT2300A::init_direct_rx_() {
  if (this->pin_gpio1_ == nullptr || this->pin_dout_ == nullptr) {
    ESP_LOGD(TAG, "init_direct_rx_: pin_gpio1=%p pin_dout=%p — need both for Direct mode",
             this->pin_gpio1_, this->pin_dout_);
    return false;
  }

  // Setup DOUT pin as input
  this->pin_dout_->setup();
  this->pin_dout_->pin_mode(gpio::FLAG_INPUT);

  // Cache raw GPIO number for fast ISR access
  this->drx_.dout_gpio_num = static_cast<gpio_num_t>(this->pin_dout_->get_pin());
  ESP_LOGV(TAG, "init_direct_rx_: DOUT on GPIO%d", (int) this->drx_.dout_gpio_num);

  // Create esp_timer for bit sampling (one-shot, chained for precision)
  esp_timer_create_args_t timer_args = {};
  timer_args.callback = sample_timer_cb_;
  timer_args.arg = this;
  timer_args.dispatch_method = ESP_TIMER_TASK;  // TASK dispatch (ISR needs CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD)
  timer_args.name = "cmt_rx_bit";

  esp_err_t err = esp_timer_create(&timer_args, &this->sample_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "init_direct_rx_: esp_timer_create failed: %d", err);
    return false;
  }

  // Setup GPIO1 (NIRQ = INT1) for SYNC_OK interrupt
  this->pin_gpio1_->setup();
  this->pin_gpio1_->pin_mode(gpio::FLAG_INPUT);
  this->pin_gpio1_->attach_interrupt(sync_ok_isr_, this, gpio::INTERRUPT_RISING_EDGE);

  memset(&this->drx_.frame_buf, 0, sizeof(this->drx_.frame_buf));
  this->drx_.active = false;
  this->drx_.frame_ready = false;

  this->direct_rx_enabled_ = true;
  ESP_LOGD(TAG, "Direct mode RX enabled (DOUT=GPIO%d, SYNC=GPIO1/NIRQ)", (int) this->drx_.dout_gpio_num);
  return true;
}

void CMT2300A::deinit_direct_rx_() {
  if (!this->direct_rx_enabled_)
    return;

  if (this->sample_timer_ != nullptr) {
    esp_timer_stop(this->sample_timer_);
    esp_timer_delete(this->sample_timer_);
    this->sample_timer_ = nullptr;
  }

  if (this->pin_gpio1_ != nullptr) {
    this->pin_gpio1_->detach_interrupt();
  }

  this->drx_.active = false;
  this->drx_.frame_ready = false;
  this->direct_rx_enabled_ = false;
}

int16_t CMT2300A::check_rx_direct_(uint8_t *buf, uint16_t max_len) {
  if (!this->drx_.frame_ready)
    return 0;

  uint16_t len = this->drx_.byte_count;
  if (len > max_len)
    len = max_len;

  memcpy(buf, this->drx_.frame_buf, len);

  ESP_LOGD(TAG, "RX direct: %d bytes, L=0x%02X, first: %02X %02X %02X %02X",
           len,
           len > 0 ? buf[0] : 0,
           len > 0 ? buf[0] : 0, len > 1 ? buf[1] : 0,
           len > 2 ? buf[2] : 0, len > 3 ? buf[3] : 0);

  // Reset state and re-arm RX (lightweight — no full config reload)
  this->drx_.frame_ready = false;
  this->drx_.active = false;
  this->drx_.byte_count = 0;
  this->clear_interrupts_();
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);

  return len;
}

#else  // !USE_ESP32

bool CMT2300A::init_direct_rx_() { return false; }
void CMT2300A::deinit_direct_rx_() {}
int16_t CMT2300A::check_rx_direct_(uint8_t *buf, uint16_t max_len) { return 0; }

#endif  // USE_ESP32

// ============================================================================
// Initialization
// ============================================================================

bool CMT2300A::init(uint8_t channel) {
  ESP_LOGD(TAG, "Initializing CMT2300A, channel %d (%.3f MHz)", channel, CMT2300A_FREQ_MHZ[channel < 4 ? channel : 1]);

  // Setup GPIO pins
  ESP_LOGV(TAG, "init: setting up GPIO pins");
  ESP_LOGVV(TAG, "init: pin_sdio_=%p pin_sclk_=%p pin_csb_=%p pin_fcsb_=%p pin_gpio1_=%p",
            this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio1_);
  if (this->pin_sdio_ == nullptr || this->pin_sclk_ == nullptr ||
      this->pin_csb_ == nullptr || this->pin_fcsb_ == nullptr) {
    ESP_LOGE(TAG, "init: one or more SPI pins are null!");
    return false;
  }

  this->pin_sdio_->setup();
  this->pin_sclk_->setup();
  this->pin_csb_->setup();
  this->pin_fcsb_->setup();
  ESP_LOGVV(TAG, "init: pin setup() done");

  this->pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_sclk_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_csb_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_fcsb_->pin_mode(gpio::FLAG_OUTPUT);
  ESP_LOGVV(TAG, "init: pin_mode(OUTPUT) done");

  // Set idle states
  this->pin_csb_->digital_write(true);
  this->pin_fcsb_->digital_write(true);
  this->pin_sclk_->digital_write(false);
  ESP_LOGVV(TAG, "init: idle states set (CSB=1, FCSB=1, SCLK=0)");

  // Allow SPI interface to sync (firmware sends 10 clock pulses; delay is equivalent)
  delay(1);

  if (this->pin_gpio1_ != nullptr) {
    ESP_LOGV(TAG, "init: GPIO1 pin provided, setting up as input");
    this->pin_gpio1_->setup();
    this->pin_gpio1_->pin_mode(gpio::FLAG_INPUT);
  } else {
    ESP_LOGV(TAG, "init: GPIO1 pin is null — ISR mode will not be available");
  }

  // Soft reset: firmware writes 0xFF to CUS_SOFTRST, then go_standby
  // See firmware function 0x13C7C
  ESP_LOGV(TAG, "init: soft reset (writing 0xFF to CUS_SOFTRST)");
  this->write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(10);
  ESP_LOGV(TAG, "init: sending GO_STBY after reset");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  delay(10);

  ESP_LOGV(TAG, "init: waiting for STBY mode...");
  if (!this->wait_for_mode_(CMT2300A_STA_STBY, 100)) {
    ESP_LOGE(TAG, "CMT2300A failed to enter standby mode after reset — chip not connected or wrong pins?");
    uint8_t mode_sta = this->read_reg(CMT2300A_CUS_MODE_STA);
    ESP_LOGE(TAG, "  CUS_MODE_STA=0x%02X (mode bits=0x%X)", mode_sta, mode_sta & CMT2300A_MASK_CHIP_MODE_STA);
    return false;
  }
  ESP_LOGV(TAG, "init: STBY mode reached after reset");

  // Write RX config as default idle config
  ESP_LOGV(TAG, "init: writing RX config (96 registers, ch=%d)", channel);
  this->write_config_(CMT2300A_RX_CONFIG, channel);
  ESP_LOGV(TAG, "init: RX config written, chip should be in SLEEP");

  // write_config_ ends in SLEEP, return to STBY
  ESP_LOGV(TAG, "init: going back to STBY after config");
  if (!this->go_standby()) {
    ESP_LOGE(TAG, "CMT2300A failed to return to standby after config");
    uint8_t mode_sta = this->read_reg(CMT2300A_CUS_MODE_STA);
    ESP_LOGE(TAG, "  CUS_MODE_STA=0x%02X (mode bits=0x%X)", mode_sta, mode_sta & CMT2300A_MASK_CHIP_MODE_STA);
    return false;
  }
  ESP_LOGV(TAG, "init: STBY mode reached after config");

  ESP_LOGVV(TAG, "init: clearing interrupts and FIFO");
  this->clear_interrupts_();
  this->clear_fifo_();

  // CUS_CMT2 (reg 0x01) defaults to 0x66 in both TX and RX configs — use it
  uint8_t check = this->read_reg(CMT2300A_CUS_CMT2);
  ESP_LOGV(TAG, "init: chip verify — CUS_CMT2(reg 0x01) = 0x%02X (expect 0x66)", check);
  if (check == 0xFF || check == 0x00) {
    ESP_LOGE(TAG, "CMT2300A not responding (read 0x%02X from reg 0x01, expected 0x66) — check SPI wiring!", check);
    return false;
  }

  // Read back a few more regs for diagnostics
  uint8_t reg_mode_sta = this->read_reg(CMT2300A_CUS_MODE_STA);
  uint8_t reg_int_en = this->read_reg(CMT2300A_CUS_INT_EN);
  uint8_t reg_io_sel = this->read_reg(CMT2300A_CUS_IO_SEL);
  uint8_t reg_fifo_ctl = this->read_reg(CMT2300A_CUS_FIFO_CTL);
  ESP_LOGV(TAG, "init: post-config regs: MODE_STA=0x%02X INT_EN=0x%02X IO_SEL=0x%02X FIFO_CTL=0x%02X",
           reg_mode_sta, reg_int_en, reg_io_sel, reg_fifo_ctl);

  // Try to initialize Direct mode RX (falls back to polling if pins not available)
  ESP_LOGV(TAG, "init: attempting Direct mode RX setup...");
  if (!this->init_direct_rx_()) {
    ESP_LOGD(TAG, "Direct mode RX not available (need pin_gpio1 + pin_dout) — using polling RX");
  }

  ESP_LOGD(TAG, "CMT2300A initialized OK (reg0=0x%02X, direct_rx=%s)", check,
           this->direct_rx_enabled_ ? "yes" : "no");
  return true;
}

// ============================================================================
// TX / RX Mode Switching
// ============================================================================

bool CMT2300A::switch_tx(uint8_t channel) {
  ESP_LOGV(TAG, "switch_tx: ch=%d", channel);

  // Soft reset before config switch — firmware does this in pre_config() (0x13FEA)
  ESP_LOGVV(TAG, "switch_tx: soft reset before config");
  this->write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(10);

  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_tx: go_standby failed (pre-config)");
    return false;
  }

  this->write_config_(CMT2300A_TX_CONFIG, channel);

  // write_config_ ends in SLEEP (per firmware). Return to STBY for FIFO access.
  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_tx: go_standby failed (post-config)");
    return false;
  }

  this->clear_interrupts_();
  this->clear_fifo_();
  ESP_LOGV(TAG, "switch_tx: ready");
  return true;
}

bool CMT2300A::switch_rx(uint8_t channel) {
  ESP_LOGV(TAG, "switch_rx: ch=%d", channel);

  // Soft reset before config switch — firmware does this in pre_config() (0x13FEA)
  ESP_LOGVV(TAG, "switch_rx: soft reset before config");
  this->write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(10);

  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_rx: go_standby failed (pre-config)");
    return false;
  }

  this->write_config_(CMT2300A_RX_CONFIG, channel);

  // write_config_ ends in SLEEP (per firmware). Return to STBY for FIFO access.
  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_rx: go_standby failed (post-config)");
    return false;
  }

  this->clear_interrupts_();
  this->clear_fifo_();
  ESP_LOGV(TAG, "switch_rx: ready");
  return true;
}

// ============================================================================
// Packet TX (blocking — OK, TX is fast: ~100ms max for W-MBus frame)
// ============================================================================

bool CMT2300A::send_packet(const uint8_t *data, uint16_t len, uint8_t channel) {
  ESP_LOGD(TAG, "TX %d bytes on ch%d", len, channel);
  ESP_LOGV(TAG, "send_packet: first bytes: %02X %02X %02X %02X ...",
           len > 0 ? data[0] : 0, len > 1 ? data[1] : 0, len > 2 ? data[2] : 0, len > 3 ? data[3] : 0);

  if (!this->switch_tx(channel)) {
    ESP_LOGW(TAG, "send_packet: switch_tx failed");
    return false;
  }

  // Write data to FIFO
  ESP_LOGV(TAG, "send_packet: writing %d bytes to FIFO", len);
  this->write_fifo(data, len);

  // Set packet length (CUS_PKT7 stores payload length)
  this->write_reg(CMT2300A_CUS_PKT7, len);

  // Go TX
  this->clear_interrupts_();
  ESP_LOGV(TAG, "send_packet: sending GO_TX");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_TX);

  // Wait for TX done (poll status, ~100ms max for typical W-MBus frame)
  uint32_t start = millis();
  while (millis() - start < 200) {
    // TX_DONE flag is in CUS_INT_CLR1 (0x6A) bit 3 (read-only)
    uint8_t clr1 = this->read_reg(CMT2300A_CUS_INT_CLR1);
    if (clr1 & CMT2300A_MASK_TX_DONE_FLG) {
      ESP_LOGD(TAG, "TX complete in %ums", millis() - start);
      this->go_standby();
      return true;
    }
    // Also check mode status — if back to standby, TX is done
    uint8_t mode = this->read_reg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
    if (mode == CMT2300A_STA_STBY) {
      ESP_LOGD(TAG, "TX complete in %ums (mode=STBY)", millis() - start);
      return true;
    }
    delayMicroseconds(500);
  }

  uint8_t final_mode = this->read_reg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
  uint8_t final_flags = this->read_reg(CMT2300A_CUS_INT_FLAG);
  ESP_LOGW(TAG, "TX timeout after 200ms (mode=0x%02X, flags=0x%02X)", final_mode, final_flags);
  this->go_standby();
  return false;
}

// ============================================================================
// Blocking RX (used by sniffer mode only)
// ============================================================================

uint16_t CMT2300A::receive_packet(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms, uint8_t channel) {
  ESP_LOGVV(TAG, "receive_packet: ch=%d timeout=%ums", channel, timeout_ms);
  if (!this->switch_rx(channel)) {
    ESP_LOGW(TAG, "receive_packet: switch_rx failed");
    return 0;
  }

  // Enter RX mode
  this->clear_interrupts_();
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
  ESP_LOGVV(TAG, "receive_packet: entered RX mode, polling...");

  // Poll for packet received
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t flags = this->read_reg(CMT2300A_CUS_INT_FLAG);
    if (flags & CMT2300A_MASK_PKT_OK_FLG) {
      // Read packet length from FIFO status
      uint8_t pkt_len = this->read_reg(CMT2300A_CUS_PKT7);  // RX payload length register
      if (pkt_len == 0 || pkt_len > max_len) {
        ESP_LOGW(TAG, "RX bad length: %d (max=%d)", pkt_len, max_len);
        this->go_standby();
        return 0;
      }

      this->read_fifo(buf, pkt_len);
      ESP_LOGD(TAG, "RX %d bytes in %ums", pkt_len, millis() - start);
      ESP_LOGV(TAG, "RX first bytes: %02X %02X %02X %02X ...",
               pkt_len > 0 ? buf[0] : 0, pkt_len > 1 ? buf[1] : 0,
               pkt_len > 2 ? buf[2] : 0, pkt_len > 3 ? buf[3] : 0);
      this->go_standby();
      return pkt_len;
    }

    // Check for packet error
    if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
      ESP_LOGW(TAG, "RX packet error (flags=0x%02X) at %ums", flags, millis() - start);
      this->clear_interrupts_();
      this->clear_fifo_();
      // Re-enter RX
      this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
    }

    delay(1);
  }

  ESP_LOGVV(TAG, "receive_packet: timeout, no packet in %ums", timeout_ms);
  this->go_standby();
  return 0;
}

// ============================================================================
// Non-blocking RX API
// ============================================================================

bool CMT2300A::start_rx(uint8_t channel) {
  ESP_LOGV(TAG, "start_rx: ch=%d direct_rx=%s", channel, this->direct_rx_enabled_ ? "yes" : "no");
  if (!this->switch_rx(channel)) {
    ESP_LOGW(TAG, "start_rx: switch_rx failed");
    return false;
  }

  if (this->direct_rx_enabled_) {
    // Direct mode: INT1 fires on SYNC_OK, ISR starts bit sampling
    ESP_LOGVV(TAG, "start_rx: setting INT1 source to SYNC_OK for Direct mode");
    this->set_int1_source_(CMT2300A_INT_SEL_SYNC_OK);
    // Reset direct RX state
    this->drx_.active = false;
    this->drx_.frame_ready = false;
    this->drx_.byte_count = 0;
  }

  this->clear_interrupts_();
  ESP_LOGVV(TAG, "start_rx: sending GO_RX");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
  this->rx_active_ = true;

  ESP_LOGV(TAG, "start_rx: now listening on ch%d", channel);
  return true;
}

int16_t CMT2300A::check_rx(uint8_t *buf, uint16_t max_len) {
  if (this->direct_rx_enabled_) {
    return this->check_rx_direct_(buf, max_len);
  }
  return this->check_rx_polling_(buf, max_len);
}

void CMT2300A::stop_rx() {
  ESP_LOGV(TAG, "stop_rx: stopping RX, direct=%s", this->direct_rx_enabled_ ? "yes" : "no");
  this->rx_active_ = false;

  if (this->direct_rx_enabled_) {
    // Stop any in-progress bit sampling
#ifdef USE_ESP32
    if (this->sample_timer_ != nullptr) {
      esp_timer_stop(this->sample_timer_);
    }
#endif
    this->drx_.active = false;
    this->drx_.frame_ready = false;
    // Restore INT1 source to default
    this->set_int1_source_(CMT2300A_INT_SEL_TX_FIFO_NMTY);
  }

  this->go_standby();
}

// Polling fallback: read interrupt flags via SPI each call (~50μs)
// Note: only useful for Packet mode (not currently used in Direct mode)
int16_t CMT2300A::check_rx_polling_(uint8_t *buf, uint16_t max_len) {
  uint8_t flags = this->read_reg(CMT2300A_CUS_INT_FLAG);

  if (flags & CMT2300A_MASK_PKT_OK_FLG) {
    // Read packet length from PKT7 register
    uint8_t pkt_len = this->read_reg(CMT2300A_CUS_PKT7);
    if (pkt_len == 0 || pkt_len > max_len)
      pkt_len = max_len;
    this->read_fifo(buf, pkt_len);
    ESP_LOGD(TAG, "RX %d bytes (polling), first: %02X %02X %02X %02X",
             pkt_len,
             pkt_len > 0 ? buf[0] : 0, pkt_len > 1 ? buf[1] : 0,
             pkt_len > 2 ? buf[2] : 0, pkt_len > 3 ? buf[3] : 0);
    this->clear_interrupts_();
    this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);  // re-enter RX
    return pkt_len;
  }

  if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
    ESP_LOGW(TAG, "RX packet error (polling, flags=0x%02X)", flags);
    this->clear_interrupts_();
    this->clear_fifo_();
    this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);  // re-enter RX
  }

  return 0;  // nothing yet
}

}  // namespace esphome::nartis_wmbus
