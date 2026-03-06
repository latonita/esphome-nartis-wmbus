#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "cmt2300a.h"
#include "nartis_wmbus_sensor.h"

namespace esphome {
namespace nartis_wmbus {

// ============================================================================
// DLMS A-XDR Data Type Tags (IEC 62056, Blue Book Table 2)
// ============================================================================
static constexpr uint8_t DLMS_TYPE_BOOLEAN        = 0x03;
static constexpr uint8_t DLMS_TYPE_INT8           = 0x0F;
static constexpr uint8_t DLMS_TYPE_UINT8          = 0x11;
static constexpr uint8_t DLMS_TYPE_INT16          = 0x10;
static constexpr uint8_t DLMS_TYPE_UINT16         = 0x12;
static constexpr uint8_t DLMS_TYPE_INT32          = 0x05;
static constexpr uint8_t DLMS_TYPE_UINT32         = 0x06;
static constexpr uint8_t DLMS_TYPE_INT64          = 0x14;
static constexpr uint8_t DLMS_TYPE_UINT64         = 0x15;
static constexpr uint8_t DLMS_TYPE_FLOAT32        = 0x17;
static constexpr uint8_t DLMS_TYPE_FLOAT64        = 0x18;
static constexpr uint8_t DLMS_TYPE_ENUM           = 0x16;
static constexpr uint8_t DLMS_TYPE_OCTET_STRING   = 0x09;
static constexpr uint8_t DLMS_TYPE_VISIBLE_STRING = 0x0A;
static constexpr uint8_t DLMS_TYPE_UTF8_STRING    = 0x0C;
static constexpr uint8_t DLMS_TYPE_DATETIME       = 0x19;  // tagged datetime (not used in GET.response data)

// DLMS APDU tags
static constexpr uint8_t DLMS_TAG_GET_RESPONSE    = 0xC4;

// ============================================================================
// W-MBus Constants
// ============================================================================
static constexpr uint8_t WMBUS_C_SND_NR = 0x44;   // Send, No Reply expected
static constexpr uint8_t WMBUS_C_SND_IR = 0x46;   // Send, Install Request
static constexpr uint8_t WMBUS_C_RSP_UD = 0x08;   // Response, User Data
static constexpr uint8_t WMBUS_CI_ENC = 0xBD;     // Encrypted DLMS payload
static constexpr uint8_t WMBUS_CI_PLAIN = 0xC5;   // Unencrypted command
static constexpr uint8_t WMBUS_SC_GCM = 0x94;     // AES-GCM-128 encrypted+compressed
static constexpr uint8_t WMBUS_BLOCK_SIZE = 16;    // Data bytes per CRC block
static constexpr uint8_t WMBUS_FIRST_BLOCK = 10;   // First block: L+C+M+A = 10 bytes

// Buffer sizes
static constexpr uint16_t MAX_FRAME_SIZE = 512;
static constexpr uint16_t MAX_APDU_SIZE = 300;

// Timeouts (ms)
static constexpr uint32_t AARE_TIMEOUT_MS = 3000;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 5000;
static constexpr uint32_t RELEASE_TIMEOUT_MS = 2000;
static constexpr uint32_t SESSION_TIMEOUT_MS = 5000;
static constexpr uint8_t MAX_RETRIES = 3;

// ============================================================================
// Hardcoded Configuration
// ============================================================================

// W-MBus identity (our reader device)
static constexpr uint16_t OUR_MANUFACTURER = 0x3832;  // "NAR" (Nartis)
static constexpr uint8_t OUR_ADDRESS[4] = {0x00, 0x00, 0x01, 0x00};
static constexpr uint8_t OUR_VERSION = 0x01;
static constexpr uint8_t OUR_DEVICE_TYPE = 0x00;

// Default DLMS system title for our CIU (EEPROM factory placeholder)
// Configurable via yaml system_title option
static constexpr uint8_t DEFAULT_SYSTEM_TITLE[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

// DLMS credentials
static constexpr uint8_t DLMS_CLIENT_SAP = 0x20;      // Meter reader association
static constexpr uint16_t DLMS_SERVER_SAP = 0x0001;
static constexpr char DLMS_PASSWORD[] = "123456";       // Default LLS password

// ============================================================================
// DLMS APDU Templates
// ============================================================================

// AARQ: Application-context=LN-no-ciphering, mechanism=LLS, password="123456"
static const uint8_t AARQ_TEMPLATE[] = {
    0x60, 0x34,                         // AARQ tag + length (52 bytes)
    // --- application-context-name [1] ---
    0xA1, 0x09, 0x06, 0x07,
    0x60, 0x85, 0x74, 0x05, 0x08, 0x01, 0x01,  // LN-no-ciphering {2.16.756.5.8.1.1}
    // --- sender-acse-requirements [10] ---
    0x8A, 0x02, 0x07, 0x80,            // BIT STRING: authentication functional unit
    // --- mechanism-name [11] ---
    0x8B, 0x07,
    0x60, 0x85, 0x74, 0x05, 0x08, 0x02, 0x01,  // LLS {2.16.756.5.8.2.1}
    // --- calling-authentication-value [12] ---
    0xAC, 0x08, 0x80, 0x06,            // GraphicString, 6 bytes
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36,  // "123456"
    // --- user-information [30]: xDLMS-Initiate ---
    0xBE, 0x10, 0x04, 0x0E,            // OCTET STRING, 14 bytes
    0x01,                               // initiate-request tag
    0x00,                               // dedicated-key: absent
    0x00,                               // response-allowed: default (TRUE)
    0x00,                               // proposed-quality-of-service: absent
    0x07,                               // proposed-dlms-version-number = 7
    0x5F, 0x1F, 0x04, 0x00,            // proposed-conformance: [31] BIT STRING, 4 bytes
    0x00, 0x7E, 0x1F,                   // LN: get,set,selective-access,block-transfer,action
    0x01, 0x2C,                         // client-max-receive-pdu-size = 300
};

// RLRQ (Release Request)
static const uint8_t RLRQ_TEMPLATE[] = {
    0x62, 0x00,  // RLRQ tag + length (minimal)
};

// ============================================================================
// Component
// ============================================================================

// Operating mode
enum class Mode : uint8_t {
  SESSION = 0,   // Active DLMS session (AARQ → GET requests → RLRQ)
  LISTEN = 1,    // Passive: wait for meter push, log all, publish matching sensors
  SNIFFER = 2,   // Raw packet dump, no sensor publishing
};

using SensorMap = std::multimap<std::string, NartisWmbusSensorBase *>;

class NartisWmbusComponent : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Pin setters (from Python config)
  void set_pin_sdio(GPIOPin *pin) { pin_sdio_ = pin; }
  void set_pin_sclk(GPIOPin *pin) { pin_sclk_ = pin; }
  void set_pin_csb(GPIOPin *pin) { pin_csb_ = pin; }
  void set_pin_fcsb(GPIOPin *pin) { pin_fcsb_ = pin; }
  void set_pin_gpio1(InternalGPIOPin *pin) { pin_gpio1_ = pin; }
  void set_channel(uint8_t ch) { channel_ = ch; }
  void set_decryption_key(const std::array<uint8_t, 16> &key) { decryption_key_ = key; }
  void set_system_title(const std::array<uint8_t, 8> &title) { system_title_ = title; }
  void set_mode(uint8_t m) { mode_ = static_cast<Mode>(m); }

  void register_sensor(NartisWmbusSensorBase *sensor);

 protected:
  // FSM
  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    INIT_RADIO,
    SEND_AARQ,
    WAIT_AARE,
    DATA_REQUEST,
    WAIT_RESPONSE,
    DATA_NEXT,
    SEND_RELEASE,
    WAIT_RELEASE,
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

  bool check_timeout_() const { return millis() - wait_.start_time >= wait_.timeout_ms; }
  void start_timeout_(uint32_t ms) { wait_.start_time = millis(); wait_.timeout_ms = ms; }

  // Session watchdog
  uint32_t session_start_ms_{0};
  bool check_session_timeout_() const { return millis() - session_start_ms_ >= SESSION_TIMEOUT_MS; }

  // Radio
  CMT2300A radio_;
  GPIOPin *pin_sdio_{nullptr};
  GPIOPin *pin_sclk_{nullptr};
  GPIOPin *pin_csb_{nullptr};
  GPIOPin *pin_fcsb_{nullptr};
  InternalGPIOPin *pin_gpio1_{nullptr};
  uint8_t channel_{1};
  Mode mode_{Mode::SESSION};

  // Encryption key and system title
  std::array<uint8_t, 16> decryption_key_{};
  std::array<uint8_t, 8> system_title_{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

  // DLMS session state
  uint8_t meter_system_title_[8]{};
  bool system_title_valid_{false};
  uint32_t invocation_counter_{0};
  uint8_t access_nr_{0};
  uint8_t retry_count_{0};

  // Sensors
  SensorMap sensors_;
  SensorMap::iterator request_iter_{};
  std::string current_obis_;

  // Buffers
  uint8_t tx_buf_[MAX_FRAME_SIZE]{};
  uint8_t rx_buf_[MAX_FRAME_SIZE]{};
  uint8_t apdu_buf_[MAX_APDU_SIZE]{};

  // CRC-16/EN-13757 (poly 0x3D65, non-reflected)
  static uint16_t crc16_en13757(const uint8_t *data, uint16_t len);

  // W-MBus frame build/parse
  uint16_t wmbus_frame_build_(uint8_t c_field, uint8_t ci_field,
                               const uint8_t *payload, uint16_t pay_len,
                               uint8_t *out);
  uint16_t wmbus_frame_parse_(const uint8_t *frame, uint16_t frame_len,
                               uint8_t *out);

  // W-MBus encrypt/decrypt
  uint16_t wmbus_encrypt_(const uint8_t *dlms_apdu, uint16_t apdu_len, uint8_t *out);
  uint16_t wmbus_decrypt_(const uint8_t *enc_payload, uint16_t enc_len, uint8_t *dlms_out);

  // AES-128-GCM (mbedtls)
  // AES-128-GCM encrypt/decrypt with AAD and auth tag support
  // tag_buf: on encrypt, receives generated tag; on decrypt, contains expected tag
  // tag_len: 0 = no tag (SC=0x94), 12 = standard GCM tag (SC with bit 0x20)
  bool aes_gcm_crypt_(bool encrypt, const uint8_t *key, const uint8_t *nonce,
                       const uint8_t *aad, uint16_t aad_len,
                       const uint8_t *input, uint16_t len, uint8_t *output,
                       uint8_t *tag_buf = nullptr, uint8_t tag_len = 0);

  // DLMS APDU helpers
  uint16_t build_get_request_(const uint8_t obis[6], uint16_t class_id,
                               uint8_t attr, uint8_t *out);
  bool parse_aare_(const uint8_t *data, uint16_t len);
  bool parse_get_response_(const uint8_t *data, uint16_t len,
                           float &value, std::string &text_value, bool &is_text);

  // High-level TX/RX
  bool transmit_dlms_(const uint8_t *apdu, uint16_t apdu_len,
                       uint8_t c_field, bool encrypt);
  uint16_t receive_dlms_(uint8_t *dlms_out, uint32_t timeout_ms);  // blocking (sniffer only)
  uint16_t process_rx_frame_(const uint8_t *rf_buf, uint16_t rf_len, uint8_t *dlms_out);  // non-blocking parse

  // Invocation counter resets to 0 each boot (no NVS persistence needed)

  // Sniffer
  void sniff_loop_();
  void log_raw_frame_(const uint8_t *data, uint16_t len);
  void log_parsed_frame_(const uint8_t *frame, uint16_t len);
  static void hex_to_str_(const uint8_t *data, uint16_t len, char *out, uint16_t out_size);
  static const LogString *c_field_to_string_(uint8_t c_field);
  static const LogString *ci_field_to_string_(uint8_t ci_field);
  static void decode_manufacturer_(uint16_t m_field, char out[4]);
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

}  // namespace nartis_wmbus
}  // namespace esphome
