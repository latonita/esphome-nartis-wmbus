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

// Maximum captured over-the-air packet size.
static constexpr uint16_t CMT_MAX_PKT_SIZE = 300;

// Firmware fixup sets FIFO threshold to 15 bytes.
static constexpr uint8_t CMT_FIFO_THRESHOLD = 15;

// Sniffer fallback path uses a fixed payload length capped by merged FIFO size.
static constexpr uint8_t CMT_SNIFF_PAYLOAD_LEN = 64;

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
    // Packet mode baseline. RX profile-specific PKT14/15 values are patched at runtime.
    0x12,
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
    0x00,   // PKT14 runtime override
    0x40,   // PKT15 runtime override
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

enum class RxProfile : uint8_t {
  METER = 0,
  SNIFF = 1,
};

struct RxCaptureState {
  uint8_t data[CMT_MAX_PKT_SIZE];
  uint16_t len;
  uint16_t expected_len;
  bool collecting;
};

static inline uint16_t wmbus_air_frame_len(uint8_t l_field) {
  uint16_t data_len = static_cast<uint16_t>(l_field) + 1;
  if (data_len <= 10)
    return data_len + 2;
  uint16_t remaining = data_len - 10;
  uint16_t blocks = (remaining + 15) / 16;
  return data_len + (1 + blocks) * 2;
}

class CMT2300A {
 public:
  void set_pins(GPIOPin *sdio, GPIOPin *sclk, GPIOPin *csb, GPIOPin *fcsb, InternalGPIOPin *gpio1,
                InternalGPIOPin *gpio3 = nullptr) {
    this->pin_sdio_ = sdio;
    this->pin_sclk_ = sclk;
    this->pin_csb_ = csb;
    this->pin_fcsb_ = fcsb;
    this->pin_gpio1_ = gpio1;
    this->pin_gpio3_ = gpio3;
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
  bool start_rx(uint8_t channel, RxProfile profile = RxProfile::METER);
  int16_t check_rx(uint8_t *buf, uint16_t max_len);  // >0=pkt_len, 0=nothing yet, -1=error
  void stop_rx();

  bool go_standby();

  bool has_isr() const { return this->isr_enabled_; }
  bool has_meter_capture() const { return this->full_meter_capture_; }

 protected:
  void spi_write_byte_(uint8_t byte);
  uint8_t spi_read_byte_();
  void write_config_(const uint8_t *config, uint8_t channel);
  void apply_fixups_(bool is_tx);
  void configure_rx_profile_(RxProfile profile);
  bool wait_for_mode_(uint8_t expected_mode, uint32_t timeout_ms = 50);
  void clear_interrupts_();
  void clear_fifo_();
  void reset_rx_capture_();
  bool append_fifo_bytes_(uint16_t count);
  bool finish_meter_packet_();

  // ISR-driven RX
  bool init_isr_();
  void deinit_isr_();
  void set_int1_source_(uint8_t source);
  int16_t check_rx_polling_(uint8_t *buf, uint16_t max_len);
  int16_t check_rx_isr_(uint8_t *buf, uint16_t max_len);

#ifdef USE_ESP32
  static void receiver_task_(void *arg);
  static void IRAM_ATTR gpio1_isr_(CMT2300A *arg);
  static void IRAM_ATTR gpio3_isr_(CMT2300A *arg);

  TaskHandle_t rx_task_handle_{nullptr};
  QueueHandle_t rx_queue_{nullptr};
#endif

  GPIOPin *pin_sdio_{nullptr};
  GPIOPin *pin_sclk_{nullptr};
  GPIOPin *pin_csb_{nullptr};
  GPIOPin *pin_fcsb_{nullptr};
  InternalGPIOPin *pin_gpio1_{nullptr};
  InternalGPIOPin *pin_gpio3_{nullptr};

  bool isr_enabled_{false};
  bool full_meter_capture_{false};
  bool rx_active_{false};
  RxProfile rx_profile_{RxProfile::METER};
  RxCaptureState rx_capture_{};
};

}  // namespace esphome::nartis_wmbus
