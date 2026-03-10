#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include <array>
#include <cstdint>
#include <cstring>

#include "cmt2300a.h"
#include "dlms_layer.h"
#include "wmbus_transport.h"
#include "sensors_registry.h"
#include "nartis_wmbus_sensor.h"

namespace esphome::nartis_wmbus {

// Timeouts (ms)
static constexpr uint32_t RADIO_INIT_DELAY_MS = 10000;  // defer radio init so WiFi/logging are ready
static constexpr uint32_t INSTALL_TIMEOUT_MS = 3000;     // SND-IR install reply wait
static constexpr uint32_t AARE_TIMEOUT_MS = 3000;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 5000;
static constexpr uint32_t SESSION_TIMEOUT_MS = 10000;  // overall session watchdog (install+AARQ+data)
static constexpr uint8_t MAX_RETRIES = 3;

// ============================================================================
// Component
// ============================================================================

// Operating mode
enum class Mode : uint8_t {
  SESSION = 0,  // Active DLMS session (AARQ → GET requests → RLRQ)
  LISTEN = 1,   // Passive: wait for meter push, log all, publish matching sensors
  SNIFFER = 2,  // Raw packet dump, no sensor publishing
};

class NartisWmbusComponent : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Pin setters (from Python config)
  void set_pin_sdio(GPIOPin *pin) { this->pin_sdio_ = pin; }
  void set_pin_sclk(GPIOPin *pin) { this->pin_sclk_ = pin; }
  void set_pin_csb(GPIOPin *pin) { this->pin_csb_ = pin; }
  void set_pin_fcsb(GPIOPin *pin) { this->pin_fcsb_ = pin; }
  void set_pin_gpio1(InternalGPIOPin *pin) { this->pin_gpio1_ = pin; }
  void set_pin_gpio3(InternalGPIOPin *pin) { this->pin_gpio3_ = pin; }
  void set_channel(uint8_t ch) { this->channel_ = ch; }
  void set_decryption_key(const std::array<uint8_t, 16> &key) { this->decryption_key_ = key; }
  void set_meter_id(const std::string &id) {
    strncpy(this->meter_id_, id.c_str(), sizeof(this->meter_id_) - 1);
    this->meter_id_[sizeof(this->meter_id_) - 1] = '\0';
  }
  void set_meter_system_title(const std::array<uint8_t, 8> &title) {
    this->configured_meter_sys_title_ = title;
    this->meter_sys_title_configured_ = true;
  }
  void set_mode(uint8_t m) { this->mode_ = static_cast<Mode>(m); }
  void set_aggressive_reconnect(bool v) { this->aggressive_reconnect_ = v; }

  void register_sensor(NartisWmbusSensorBase *sensor);

 protected:
  // FSM
  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    INIT_SESSION,
    SEND_INSTALL,  // Send SND-IR install request (pairing beacon)
    WAIT_INSTALL,  // Wait for meter's install reply
    SEND_AARQ,     // Send DLMS AARQ (association request)
    WAIT_AARE,
    DATA_REQUEST,
    WAIT_RESPONSE,
    DATA_NEXT,
    PUBLISH,
    LISTENING,
    SNIFFING,
  } state_{State::NOT_INITIALIZED};

  void set_next_state_(State next_state);

  // Wait helper
  struct {
    uint32_t start_time{0};
    uint32_t timeout_ms{0};
  } wait_{};

  bool check_timeout_() const { return millis() - this->wait_.start_time >= this->wait_.timeout_ms; }
  void start_timeout_(uint32_t ms) {
    this->wait_.start_time = millis();
    this->wait_.timeout_ms = ms;
  }

  // Session watchdog
  uint32_t session_start_ms_{0};
  bool check_session_timeout_() const { return millis() - this->session_start_ms_ >= SESSION_TIMEOUT_MS; }

  // Radio
  CMT2300A radio_;
  GPIOPin *pin_sdio_{nullptr};
  GPIOPin *pin_sclk_{nullptr};
  GPIOPin *pin_csb_{nullptr};
  GPIOPin *pin_fcsb_{nullptr};
  InternalGPIOPin *pin_gpio1_{nullptr};
  InternalGPIOPin *pin_gpio3_{nullptr};
  uint8_t channel_{1};
  Mode mode_{Mode::SESSION};
  bool aggressive_reconnect_{false};

  // Encryption key and system titles
  std::array<uint8_t, 16> decryption_key_{};
  std::array<uint8_t, 8> system_title_{};                // Our system title (auto-generated from MAC)
  char meter_id_[13]{};                                  // 12-digit meter serial from label (optional, for logging)
  std::array<uint8_t, 8> configured_meter_sys_title_{};  // Meter's system title from YAML
  bool meter_sys_title_configured_{false};

  // DLMS session state
  uint8_t meter_system_title_[8]{};
  bool system_title_valid_{false};
  bool associated_{false};
  uint32_t invocation_counter_{0};
  uint8_t access_nr_{0};
  uint8_t retry_count_{0};

  SensorRegistry registry_;

  // Buffers
  uint8_t tx_buf_[MAX_FRAME_SIZE]{};
  uint8_t rx_buf_[MAX_FRAME_SIZE]{};
  uint8_t apdu_buf_[MAX_APDU_DECOMPRESSED]{};

  // W-MBus Install Request (SND-IR pairing beacon, per firmware 0x101DC)
  bool send_install_frame_();

  // High-level TX/RX
  bool transmit_dlms_(const uint8_t *apdu, uint16_t apdu_len, uint8_t c_field, bool encrypt);
  uint16_t receive_dlms_(uint8_t *dlms_out, uint32_t timeout_ms);                         // blocking (sniffer only)
  uint16_t process_rx_frame_(const uint8_t *rf_buf, uint16_t rf_len, uint8_t *dlms_out);  // non-blocking parse

  // Invocation counter resets to 0 each boot (no NVS persistence needed)

  // Deferred radio init (runs after WiFi/logging stabilize)
  void deferred_radio_init_();

  // Session FSM helpers
  void abort_session_to_idle_();
  bool start_rx_with_timeout_(uint32_t timeout_ms, State next_state, const char *failure_log);
  void handle_init_session_();
  void handle_send_install_();
  void handle_wait_install_();
  void handle_send_aarq_();
  void handle_wait_aare_();
  void handle_data_request_();
  void handle_wait_response_();
  void handle_data_next_();
  void handle_publish_();

  // Sniffer
  void sniff_loop_();
  void log_raw_frame_(const uint8_t *data, uint16_t len);
  void log_parsed_frame_(const uint8_t *frame, uint16_t len);
  static void hex_to_str_(const uint8_t *data, uint16_t len, char *out, uint16_t out_size);
  static const LogString *c_field_to_string_(uint8_t c_field);
  static const LogString *ci_field_to_string_(uint8_t ci_field);
  uint32_t sniffer_packet_count_{0};

  // Listen mode
  void listen_loop_();
  void listen_parse_dlms_(const uint8_t *data, uint16_t len);
  void scan_obis_values_(const uint8_t *data, uint16_t len);
  uint32_t listen_packet_count_{0};
  static const LogString *mode_to_string_(Mode mode);

  // Logging
  static const LogString *state_to_string_(State state);
};

}  // namespace esphome::nartis_wmbus
