#pragma once

#include "esphome/core/hal.h"

#include <cstdint>

#include "cmt2300a_defs.h"

#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
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

// Maximum packet size for RX queue
static constexpr uint16_t CMT_MAX_PKT_SIZE = 255;

// RX fixed payload length (must match PKT14/PKT15 in RX config, capped at merged FIFO size)
static constexpr uint8_t CMT_RX_PAYLOAD_LEN = 64;

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
    // NOTE: Original firmware uses Direct mode (0x10) — MCU reads raw bits from GPIO.
    // We use Packet mode (0x12) so the chip handles preamble/sync and fills FIFO.
    // Payload capped at 64 bytes (merged FIFO size) — captures W-MBus header + first blocks.
    // TODO: implement FIFO threshold draining for full-frame (>64 byte) capture.
    0x12,  // PKT1: was 0x10 (Direct), now Packet mode (DATA_MODE=0x02), preamble detect unchanged
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
    0xD4,
    0x2D,
    0x00,   // PKT14: PAYLOAD_LENG_10_8=0, PKT_TYPE=Fixed
    0x40,   // PKT15: PAYLOAD_LENG_7_0=64 (match merged FIFO size)
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
  uint8_t len;
};

class CMT2300A {
 public:
  void set_pins(GPIOPin *sdio, GPIOPin *sclk, GPIOPin *csb, GPIOPin *fcsb, InternalGPIOPin *gpio1) {
    this->pin_sdio_ = sdio;
    this->pin_sclk_ = sclk;
    this->pin_csb_ = csb;
    this->pin_fcsb_ = fcsb;
    this->pin_gpio1_ = gpio1;
  }

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
  // Uses ISR + FreeRTOS queue when GPIO1 is available, falls back to polling otherwise
  bool start_rx(uint8_t channel);
  int16_t check_rx(uint8_t *buf, uint16_t max_len);  // >0=pkt_len, 0=nothing yet, -1=error
  void stop_rx();

  bool go_standby();

  bool has_isr() const { return this->isr_enabled_; }

 protected:
  void spi_write_byte_(uint8_t byte);
  uint8_t spi_read_byte_();
  void write_config_(const uint8_t *config, uint8_t channel);
  void apply_fixups_();
  bool wait_for_mode_(uint8_t expected_mode, uint32_t timeout_ms = 50);
  void clear_interrupts_();
  void clear_fifo_();

  // ISR-driven RX
  bool init_isr_();
  void deinit_isr_();
  void set_int1_source_(uint8_t source);
  int16_t check_rx_polling_(uint8_t *buf, uint16_t max_len);
  int16_t check_rx_isr_(uint8_t *buf, uint16_t max_len);

#ifdef USE_ESP32
  static void receiver_task_(void *arg);
  static void IRAM_ATTR gpio1_isr_(CMT2300A *arg);

  TaskHandle_t rx_task_handle_{nullptr};
  QueueHandle_t rx_queue_{nullptr};
#endif

  GPIOPin *pin_sdio_{nullptr};
  GPIOPin *pin_sclk_{nullptr};
  GPIOPin *pin_csb_{nullptr};
  GPIOPin *pin_fcsb_{nullptr};
  InternalGPIOPin *pin_gpio1_{nullptr};

  bool isr_enabled_{false};
  bool rx_active_{false};
};

}  // namespace esphome::nartis_wmbus
