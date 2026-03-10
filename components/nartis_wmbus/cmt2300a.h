#pragma once

#include "esphome/core/hal.h"

#include <cstdint>

#include "cmt2300a_defs.h"

#ifdef USE_ESP32
#include <esp_timer.h>
#include <driver/gpio.h>
#endif

namespace esphome::nartis_wmbus {

// Register bank sizes (96 total)
static constexpr uint8_t CMT_BANK_CUS_CMT = 12;        // regs 0x00-0x0B
static constexpr uint8_t CMT_BANK_CUS_SYS = 12;        // regs 0x0C-0x17
static constexpr uint8_t CMT_BANK_CUS_FREQ = 8;        // regs 0x18-0x1F
static constexpr uint8_t CMT_BANK_CUS_DATA_RATE = 24;  // regs 0x20-0x37
static constexpr uint8_t CMT_BANK_CUS_BASEBAND = 29;   // regs 0x38-0x54
static constexpr uint8_t CMT_BANK_CUS_TX = 11;         // regs 0x55-0x5F
static constexpr uint8_t CMT_TOTAL_REGS = 96;

// Maximum packet size for RX queue (W-MBus frames with CRC can be ~291 bytes)
static constexpr uint16_t CMT_MAX_PKT_SIZE = 300;

// Direct mode RX constants
static constexpr uint32_t DIRECT_RX_BIT_PERIOD_US = 417;   // 1000000 / 2400 bps
static constexpr uint32_t DIRECT_RX_HALF_BIT_US = 208;     // center of first bit cell
static constexpr uint32_t DIRECT_RX_TIMEOUT_BITS = 2500;   // ~1.04s, max 300 bytes = 2400 bits
static constexpr uint16_t DIRECT_RX_MAX_FRAME = 300;

// TX mode register config (ROM 0x1373C, 96 bytes)
static const uint8_t CMT2300A_TX_CONFIG[CMT_TOTAL_REGS] = {
    // CUS_CMT (0x00-0x0B)
    0x00,
    0x66,
    0xEC,
    0x1D,
    0xF0,
    0x80,
    0x14,
    0x08,
    0x91,
    0x02,
    0x02,
    0xA0,
    // CUS_SYS (0x0C-0x17)
    0xAE,
    0xE0,
    0x35,
    0x00,
    0x00,
    0xF4,
    0x10,
    0xE2,
    0x42,
    0x20,
    0x00,
    0x81,
    // CUS_FREQ (0x18-0x1F) — default Ch0, replaced at runtime
    0x42,
    0x6D,
    0x8F,
    0x1C,
    0x42,
    0x57,
    0xDD,
    0x1B,
    // CUS_DATA_RATE (0x20-0x37)
    0x32,
    0x18,
    0x00,
    0x99,
    0xC1,
    0x9B,
    0x07,
    0x0A,
    0x9F,
    0x39,
    0x29,
    0x29,
    0xC0,
    0x51,
    0x2A,
    0x53,
    0x00,
    0x00,
    0xB4,
    0x00,
    0x00,
    0x01,
    0x00,
    0x00,
    // CUS_BASEBAND (0x38-0x54)
    0x2A,
    0x0A,
    0x00,
    0x55,
    0x06,
    0x00,
    0x00,
    0x00,
    0x00,
    0xF6,
    0x55,
    0x55,
    0x55,
    0x10,
    0xFF,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x60,
    0xFF,
    0x00,
    0x00,
    0x1F,
    0x10,
    // CUS_TX (0x55-0x5F)
    0x00,
    0x00,
    0x00,
    0x50,
    0xC7,
    0x03,
    0x00,
    0x42,
    0xB0,
    0x00,
    0x8A,
};

// RX mode register config (ROM 0x137A0, 96 bytes)
static const uint8_t CMT2300A_RX_CONFIG[CMT_TOTAL_REGS] = {
    // CUS_CMT (0x00-0x0B)
    0x00,
    0x66,
    0xEC,
    0x1D,
    0xF0,
    0x80,
    0x14,
    0x08,
    0x11,
    0x02,
    0x02,
    0x00,
    // CUS_SYS (0x0C-0x17)
    0xAE,
    0xE0,
    0x35,
    0x00,
    0x00,
    0xF4,
    0x10,
    0xE2,
    0x42,
    0x20,
    0x00,
    0x81,
    // CUS_FREQ (0x18-0x1F) — default Ch0, replaced at runtime
    0x42,
    0x6D,
    0x8F,
    0x1C,
    0x42,
    0x57,
    0xDD,
    0x1B,
    // CUS_DATA_RATE (0x20-0x37)
    0x32,
    0x18,
    0x80,
    0xDD,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x29,
    0xC0,
    0x51,
    0x2A,
    0x4B,
    0x05,
    0x00,
    0x50,
    0x2D,
    0x00,
    0x01,
    0x05,
    0x05,
    // CUS_BASEBAND (0x38-0x54)
    // Direct mode RX (DATA_MODE=0): chip demodulates and outputs raw bits on DOUT (GPIO3).
    // MCU samples bits via timer ISR after SYNC_OK interrupt. Matches original firmware.
    0x10,  // PKT1: Direct mode, RX preamble detect
    0x08,
    0x00,
    0xAA,
    0x02,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xD4,  // PKT12: RX sync word byte 1
    0x2D,  // PKT13: RX sync word byte 0
    0x00,  // PKT14
    0x1F,  // PKT15
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x60,
    0xFF,
    0x00,
    0x00,
    0x1F,
    0x10,
    // CUS_TX (0x55-0x5F)
    0x00,
    0x00,
    0x00,
    0x55,
    0x9A,
    0x0C,
    0x00,
    0x0F,
    0xB0,
    0x00,
    0x8A,
};

// Frequency channels (ROM 0x13804, 4 channels x 8 bytes CUS_FREQ bank)
static const uint8_t CMT2300A_FREQ_CHANNELS[4][CMT_BANK_CUS_FREQ] = {
    {0x42, 0x6D, 0x8F, 0x1C, 0x42, 0x57, 0xDD, 0x1B},  // Ch0: 431.782 MHz
    {0x42, 0xBF, 0x47, 0x1B, 0x42, 0xA9, 0x95, 0x1A},  // Ch1: 433.857 MHz (default)
    {0x42, 0xB2, 0xA4, 0x1D, 0x42, 0x9B, 0xF2, 0x1C},  // Ch2: 433.536 MHz
    {0x42, 0xF6, 0xB9, 0x1E, 0x42, 0xE0, 0x07, 0x1E},  // Ch3: 435.264 MHz
};

// Channel frequencies for logging
static const float CMT2300A_FREQ_MHZ[4] = {431.782f, 433.857f, 433.536f, 435.264f};

// RX packet passed through FreeRTOS queue
struct RxPacket {
  uint8_t data[CMT_MAX_PKT_SIZE];
  uint16_t len;
};

// Direct mode RX bit-sampling state
struct DirectRxState {
  uint8_t frame_buf[DIRECT_RX_MAX_FRAME];
  volatile uint16_t byte_count;
  volatile uint8_t bit_count;         // bits accumulated in current byte (0-7)
  volatile uint8_t current_byte;      // byte being assembled (MSB first)
  volatile uint16_t expected_air_len; // total bytes to receive (computed from L-field)
  volatile bool active;               // timer is sampling bits
  volatile bool frame_ready;          // complete frame available for check_rx
  volatile uint32_t total_bits;       // timeout counter
  gpio_num_t dout_gpio_num;           // cached GPIO number for fast read
};

// Calculate W-MBus over-the-air frame length (with interleaved CRCs) from L-field
static inline uint16_t wmbus_air_frame_len(uint8_t l_field) {
  uint16_t data_len = (uint16_t) l_field + 1;  // L-field byte + payload
  if (data_len <= 10)
    return data_len + 2;  // single CRC block
  uint16_t remaining = data_len - 10;
  uint16_t blocks = (remaining + 15) / 16;  // ceiling division
  return data_len + (1 + blocks) * 2;
}

class CMT2300A {
 public:
  void set_pins(GPIOPin *sdio, GPIOPin *sclk, GPIOPin *csb, GPIOPin *fcsb, InternalGPIOPin *gpio1) {
    this->pin_sdio_ = sdio;
    this->pin_sclk_ = sclk;
    this->pin_csb_ = csb;
    this->pin_fcsb_ = fcsb;
    this->pin_gpio1_ = gpio1;  // connected to module NIRQ (CMT2300A GPIO2 = INT1)
  }
  void set_pin_dout(InternalGPIOPin *pin) { this->pin_dout_ = pin; }  // module GPIO3 = DOUT

  bool init(uint8_t channel);

  void write_reg(uint8_t addr, uint8_t value);
  uint8_t read_reg(uint8_t addr);

  void write_bank(uint8_t start_reg, const uint8_t *data, uint8_t count);
  void write_fifo(const uint8_t *data, uint16_t len);
  void read_fifo(uint8_t *data, uint16_t len);

  bool switch_tx(uint8_t channel);
  bool switch_rx(uint8_t channel);

  bool send_packet(const uint8_t *data, uint16_t len, uint8_t channel);
  uint16_t receive_packet(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms, uint8_t channel);

  // Non-blocking RX API (for ESPHome loop()-friendly operation)
  // Uses Direct mode RX (DOUT bit sampling) when GPIO1+DOUT available, falls back to polling
  bool start_rx(uint8_t channel);
  int16_t check_rx(uint8_t *buf, uint16_t max_len);  // >0=pkt_len, 0=nothing yet, -1=error
  void stop_rx();

  bool go_standby();

  bool has_direct_rx() const { return this->direct_rx_enabled_; }

 protected:
  void spi_write_byte_(uint8_t byte);
  uint8_t spi_read_byte_();
  void write_config_(const uint8_t *config, uint8_t channel);
  void apply_fixups_(bool is_tx);
  bool wait_for_mode_(uint8_t expected_mode, uint32_t timeout_ms = 50);
  void clear_interrupts_();
  void clear_fifo_();

  // Common
  void set_int1_source_(uint8_t source);

  // Direct mode RX (matches firmware: DOUT on GPIO3, SYNC_OK on GPIO2/INT1)
  bool init_direct_rx_();
  void deinit_direct_rx_();
  int16_t check_rx_direct_(uint8_t *buf, uint16_t max_len);

#ifdef USE_ESP32
  static void IRAM_ATTR sync_ok_isr_(void *arg);
  static void IRAM_ATTR sample_timer_cb_(void *arg);

  esp_timer_handle_t sample_timer_{nullptr};
  DirectRxState drx_{};
#endif

  // Fallback polling RX (no GPIO pins needed)
  int16_t check_rx_polling_(uint8_t *buf, uint16_t max_len);

  GPIOPin *pin_sdio_{nullptr};
  GPIOPin *pin_sclk_{nullptr};
  GPIOPin *pin_csb_{nullptr};
  GPIOPin *pin_fcsb_{nullptr};
  InternalGPIOPin *pin_gpio1_{nullptr};   // module NIRQ = CMT2300A GPIO2 (INT1/SYNC_OK)
  InternalGPIOPin *pin_dout_{nullptr};    // module GPIO3 = CMT2300A GPIO3 (DOUT)

  bool direct_rx_enabled_{false};
  bool rx_active_{false};
};

}  // namespace esphome::nartis_wmbus
