#include "nartis_wmbus.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstring>

namespace esphome::nartis_wmbus {

static const char *const TAG = "nartis_wmbus";

bool NartisWmbusComponent::send_install_frame_() {
  uint8_t payload[INSTALL_PAYLOAD_SIZE];
  build_install_payload(this->meter_id_, payload);
  ESP_LOGV(TAG, "send_install_frame_: meter_id='%s', payload=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
           this->meter_id_, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5],
           payload[6], payload[7], payload[8], payload[9], payload[10], payload[11], payload[12]);

  uint8_t frame_buf[MAX_FRAME_SIZE];
  this->access_nr_++;

  uint16_t frame_len = wmbus_frame_build(WMBUS_C_SND_IR, payload, INSTALL_PAYLOAD_SIZE, this->access_nr_, frame_buf);

  ESP_LOGD(TAG, "Sending SND-IR install request (%d bytes, access_nr=%d)", frame_len, this->access_nr_);
  bool ok = this->radio_.send_packet(frame_buf, frame_len, this->channel_);
  if (!ok) {
    ESP_LOGW(TAG, "send_install_frame_: radio send_packet returned false");
  }
  return ok;
}

bool NartisWmbusComponent::transmit_dlms_(const uint8_t *apdu, uint16_t apdu_len, uint8_t c_field, bool encrypt) {
  uint8_t frame_buf[MAX_FRAME_SIZE];

  this->access_nr_++;
  ESP_LOGV(TAG, "transmit_dlms_: apdu_len=%d c_field=0x%02X encrypt=%s sys_title_valid=%s access_nr=%d",
           apdu_len, c_field, YESNO(encrypt), YESNO(this->system_title_valid_), this->access_nr_);
  ESP_LOGVV(TAG, "transmit_dlms_: apdu[0..3]=%02X %02X %02X %02X",
            apdu_len > 0 ? apdu[0] : 0, apdu_len > 1 ? apdu[1] : 0,
            apdu_len > 2 ? apdu[2] : 0, apdu_len > 3 ? apdu[3] : 0);

  if (encrypt && this->system_title_valid_) {
    ESP_LOGV(TAG, "transmit_dlms_: encrypting with IC=%u", this->invocation_counter_);
    uint8_t enc_payload[MAX_APDU_SIZE];
    uint16_t enc_len =
        dlms_encrypt(TAG, this->decryption_key_, this->system_title_, this->invocation_counter_, apdu, apdu_len,
                      enc_payload);
    if (enc_len == 0) {
      ESP_LOGW(TAG, "transmit_dlms_: encryption failed");
      return false;
    }
    ESP_LOGV(TAG, "transmit_dlms_: encrypted payload %d bytes", enc_len);
    this->invocation_counter_++;
    uint16_t frame_len = wmbus_frame_build(c_field, enc_payload, enc_len, this->access_nr_, frame_buf);
    ESP_LOGV(TAG, "transmit_dlms_: frame built %d bytes (encrypted)", frame_len);
    return this->radio_.send_packet(frame_buf, frame_len, this->channel_);
  }

  ESP_LOGV(TAG, "transmit_dlms_: sending plaintext");
  uint16_t frame_len = wmbus_frame_build(c_field, apdu, apdu_len, this->access_nr_, frame_buf);
  ESP_LOGV(TAG, "transmit_dlms_: frame built %d bytes (plaintext)", frame_len);
  return this->radio_.send_packet(frame_buf, frame_len, this->channel_);
}

uint16_t NartisWmbusComponent::process_rx_frame_(const uint8_t *rf_buf, uint16_t rf_len, uint8_t *dlms_out) {
  ESP_LOGV(TAG, "process_rx_frame_: rf_len=%d", rf_len);
  uint8_t stripped[MAX_APDU_SIZE + 30];
  uint16_t stripped_len = wmbus_frame_parse(TAG, rf_buf, rf_len, stripped);
  if (stripped_len == 0) {
    ESP_LOGW(TAG, "Failed to parse W-MBus frame (CRC error or too short)");
    return 0;
  }

  if (stripped_len < 11) {
    ESP_LOGW(TAG, "Frame too short after CRC strip: %d", stripped_len);
    return 0;
  }

  uint8_t ci_field = stripped[10];
  uint16_t m_field = stripped[2] | (stripped[3] << 8);
  uint32_t serial = stripped[4] | (stripped[5] << 8) | (stripped[6] << 16) | (stripped[7] << 24);
  ESP_LOGD(TAG, "RX frame: L=%d C=0x%02X CI=0x%02X total=%d M=0x%04X S=%08X",
           stripped[0], stripped[1], ci_field, stripped_len, m_field, serial);

  static constexpr uint16_t DLL_HDR_LEN = 11;
  static constexpr uint16_t TPL_SHORT_HDR_LEN = 4;
  static constexpr uint16_t FULL_HDR_LEN = DLL_HDR_LEN + TPL_SHORT_HDR_LEN;

  if (ci_field == WMBUS_CI_TPL_SHORT) {
    ESP_LOGV(TAG, "process_rx_frame_: CI=TPL_SHORT (0x7A)");
    if (stripped_len < FULL_HDR_LEN) {
      ESP_LOGW(TAG, "Frame too short for TPL header: %d < %d", stripped_len, FULL_HDR_LEN);
      return 0;
    }

    uint8_t *data_after_tpl = &stripped[FULL_HDR_LEN];
    uint16_t data_len = stripped_len - FULL_HDR_LEN;

    ESP_LOGD(TAG, "  TPL: C_copy=0x%02X CI_copy=0x%02X acc=%d status=0x%02X", stripped[11], stripped[12],
             stripped[13], stripped[14]);

    if (data_len == 0) {
      ESP_LOGV(TAG, "process_rx_frame_: no data after TPL header");
      return 0;
    }

    ESP_LOGV(TAG, "process_rx_frame_: data after TPL: %d bytes, SC=0x%02X", data_len, data_after_tpl[0]);
    if (data_after_tpl[0] & DLMS_SC_ENCRYPTED) {
      ESP_LOGV(TAG, "process_rx_frame_: encrypted payload, decrypting (sys_title_valid=%s)", YESNO(this->system_title_valid_));
      return dlms_decrypt(TAG, this->decryption_key_, this->meter_system_title_, data_after_tpl, data_len, dlms_out);
    }

    ESP_LOGV(TAG, "process_rx_frame_: plaintext payload after TPL, %d bytes", data_len);
    memcpy(dlms_out, data_after_tpl, data_len);
    return data_len;
  }

  uint8_t *payload = &stripped[DLL_HDR_LEN];
  uint16_t payload_len = stripped_len - DLL_HDR_LEN;

  if (ci_field == WMBUS_CI_ENC) {
    ESP_LOGV(TAG, "process_rx_frame_: CI=ENC (0xBD), encrypted DLMS %d bytes", payload_len);
    return dlms_decrypt(TAG, this->decryption_key_, this->meter_system_title_, payload, payload_len, dlms_out);
  }
  if (ci_field == WMBUS_CI_PLAIN) {
    ESP_LOGV(TAG, "process_rx_frame_: CI=PLAIN (0xC5), plaintext %d bytes", payload_len);
    memcpy(dlms_out, payload, payload_len);
    return payload_len;
  }

  ESP_LOGW(TAG, "Unknown CI field: 0x%02X (stripped_len=%d)", ci_field, stripped_len);
  return 0;
}

uint16_t NartisWmbusComponent::receive_dlms_(uint8_t *dlms_out, uint32_t timeout_ms) {
  uint8_t rf_buf[MAX_FRAME_SIZE];
  uint16_t rf_len = this->radio_.receive_packet(rf_buf, sizeof(rf_buf), timeout_ms, this->channel_);
  if (rf_len == 0)
    return 0;
  return this->process_rx_frame_(rf_buf, rf_len, dlms_out);
}

void NartisWmbusComponent::register_sensor(NartisWmbusSensorBase *sensor) {
  this->registry_.add(sensor);
}

// ============================================================================
// Component Lifecycle
// ============================================================================

void NartisWmbusComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Nartis W-MBus component...");

  // Auto-generate our system title from ESP32 MAC: 'E' 'S' + 6 MAC bytes
  uint8_t mac[6];
  get_mac_address_raw(mac);
  this->system_title_ = {'E', 'S', mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]};
  ESP_LOGV(TAG, "setup: system title from MAC: %02X%02X%02X%02X%02X%02X%02X%02X",
           this->system_title_[0], this->system_title_[1], this->system_title_[2], this->system_title_[3],
           this->system_title_[4], this->system_title_[5], this->system_title_[6], this->system_title_[7]);

  // Initialize radio
  ESP_LOGV(TAG, "setup: setting radio pins (sdio=%p sclk=%p csb=%p fcsb=%p gpio1=%p)",
           this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio1_);
  this->radio_.set_pins(this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio1_);

  ESP_LOGV(TAG, "setup: preparing sensor registry (%d sensors)", (int) this->registry_.size());
  this->registry_.prepare_requests();

  ESP_LOGV(TAG, "setup: initializing radio (mode=%s, ch=%d)",
           LOG_STR_ARG(mode_to_string_(this->mode_)), this->channel_);

  if (!this->radio_.init(this->channel_)) {
    ESP_LOGE(TAG, "Radio init failed — component marked as failed. Check SPI wiring and pin config!");
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "setup: radio init succeeded");

  float freq = CMT2300A_FREQ_MHZ[this->channel_ < 4 ? this->channel_ : 1];
  switch (this->mode_) {
    case Mode::SNIFFER:
      ESP_LOGI(TAG, "Entering SNIFFER mode on ch %d (%.3f MHz)", this->channel_, freq);
      if (!this->radio_.start_rx(this->channel_)) {
        ESP_LOGE(TAG, "Failed to start RX for sniffer mode");
        this->mark_failed();
        return;
      }
      this->state_ = State::SNIFFING;
      break;
    case Mode::LISTEN:
      if (this->meter_sys_title_configured_) {
        memcpy(this->meter_system_title_, this->configured_meter_sys_title_.data(), 8);
        this->system_title_valid_ = true;
        ESP_LOGI(TAG, "Entering LISTEN mode on ch %d (%.3f MHz), meter_sys_title=%02X%02X%02X%02X%02X%02X%02X%02X",
                 this->channel_, freq, this->meter_system_title_[0], this->meter_system_title_[1],
                 this->meter_system_title_[2], this->meter_system_title_[3], this->meter_system_title_[4],
                 this->meter_system_title_[5], this->meter_system_title_[6], this->meter_system_title_[7]);
      } else {
        ESP_LOGW(TAG,
                 "Entering LISTEN mode on ch %d (%.3f MHz) — no meter_system_title configured, decryption disabled",
                 this->channel_, freq);
      }
      if (!this->radio_.start_rx(this->channel_)) {
        ESP_LOGE(TAG, "Failed to start RX for listen mode");
        this->mark_failed();
        return;
      }
      this->state_ = State::LISTENING;
      break;
    default:
      ESP_LOGV(TAG, "setup: SESSION mode — meter_sys_title_configured=%s, aggressive_reconnect=%s",
               YESNO(this->meter_sys_title_configured_), YESNO(this->aggressive_reconnect_));
      ESP_LOGD(TAG, "Radio ready, waiting for first poll");
      this->state_ = State::IDLE;
      break;
  }
}

void NartisWmbusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Nartis W-MBus:");
  ESP_LOGCONFIG(TAG, "  Channel: %d (%.3f MHz)", this->channel_,
                CMT2300A_FREQ_MHZ[this->channel_ < 4 ? this->channel_ : 1]);
  ESP_LOGCONFIG(TAG, "  AES Key: %02X%02X%02X%02X...%02X%02X%02X%02X", this->decryption_key_[0],
                this->decryption_key_[1], this->decryption_key_[2], this->decryption_key_[3], this->decryption_key_[12],
                this->decryption_key_[13], this->decryption_key_[14], this->decryption_key_[15]);
  ESP_LOGCONFIG(TAG, "  Our System Title: %02X%02X%02X%02X%02X%02X%02X%02X (from MAC)", this->system_title_[0],
                this->system_title_[1], this->system_title_[2], this->system_title_[3], this->system_title_[4],
                this->system_title_[5], this->system_title_[6], this->system_title_[7]);
  if (this->meter_id_[0] != '\0')
    ESP_LOGCONFIG(TAG, "  Meter ID: %s", this->meter_id_);
  if (this->meter_sys_title_configured_)
    ESP_LOGCONFIG(TAG, "  Meter System Title: %02X%02X%02X%02X%02X%02X%02X%02X", this->configured_meter_sys_title_[0],
                  this->configured_meter_sys_title_[1], this->configured_meter_sys_title_[2],
                  this->configured_meter_sys_title_[3], this->configured_meter_sys_title_[4],
                  this->configured_meter_sys_title_[5], this->configured_meter_sys_title_[6],
                  this->configured_meter_sys_title_[7]);
  else
    ESP_LOGCONFIG(TAG, "  Meter System Title: not configured (will obtain from AARE)");
  ESP_LOGCONFIG(TAG, "  DLMS Password: %s", DLMS_PASSWORD);
  ESP_LOGCONFIG(TAG, "  Client SAP: 0x%02X", DLMS_CLIENT_SAP);
  ESP_LOGCONFIG(TAG, "  Invocation Counter: %u", this->invocation_counter_);
  ESP_LOGCONFIG(TAG, "  Mode: %s", LOG_STR_ARG(mode_to_string_(this->mode_)));
  ESP_LOGCONFIG(TAG, "  Aggressive Reconnect: %s", YESNO(this->aggressive_reconnect_));
  ESP_LOGCONFIG(TAG, "  Sensors: %d", this->registry_.size());
  LOG_UPDATE_INTERVAL(this);
}

void NartisWmbusComponent::update() {
  if (this->mode_ == Mode::SNIFFER) {
    ESP_LOGD(TAG, "Sniffer: %u packets captured so far", this->sniffer_packet_count_);
    return;
  }

  if (this->mode_ == Mode::LISTEN) {
    ESP_LOGD(TAG, "Listen: %u packets received", this->listen_packet_count_);
    // TODO: publish sensor values once push data parsing is implemented
    return;
  }

  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Update skipped — not idle (state=%s)", LOG_STR_ARG(state_to_string_(this->state_)));
    return;
  }

  if (this->registry_.empty()) {
    ESP_LOGW(TAG, "No sensors configured");
    return;
  }

  ESP_LOGD(TAG, "Starting data collection session");
  this->set_next_state_(State::INIT_SESSION);
}

void NartisWmbusComponent::set_next_state_(State next_state) {
  if (this->state_ != next_state) {
    ESP_LOGD(TAG, "State: %s -> %s", LOG_STR_ARG(state_to_string_(this->state_)),
             LOG_STR_ARG(state_to_string_(next_state)));
  }
  this->state_ = next_state;
}

const LogString *NartisWmbusComponent::state_to_string_(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return LOG_STR("NOT_INITIALIZED");
    case State::IDLE:
      return LOG_STR("IDLE");
    case State::INIT_SESSION:
      return LOG_STR("INIT_SESSION");
    case State::SEND_INSTALL:
      return LOG_STR("SEND_INSTALL");
    case State::WAIT_INSTALL:
      return LOG_STR("WAIT_INSTALL");
    case State::SEND_AARQ:
      return LOG_STR("SEND_AARQ");
    case State::WAIT_AARE:
      return LOG_STR("WAIT_AARE");
    case State::DATA_REQUEST:
      return LOG_STR("DATA_REQUEST");
    case State::WAIT_RESPONSE:
      return LOG_STR("WAIT_RESPONSE");
    case State::DATA_NEXT:
      return LOG_STR("DATA_NEXT");
    case State::PUBLISH:
      return LOG_STR("PUBLISH");
    case State::LISTENING:
      return LOG_STR("LISTENING");
    case State::SNIFFING:
      return LOG_STR("SNIFFING");
    default:
      return LOG_STR("UNKNOWN");
  }
}

// ============================================================================
// Session FSM
// ============================================================================

void NartisWmbusComponent::abort_session_to_idle_() {
  ESP_LOGV(TAG, "abort_session_to_idle_: aborting, going standby");
  this->radio_.go_standby();
  this->associated_ = false;
  this->set_next_state_(State::IDLE);
}

bool NartisWmbusComponent::start_rx_with_timeout_(uint32_t timeout_ms, State next_state, const char *failure_log) {
  ESP_LOGV(TAG, "start_rx_with_timeout_: timeout=%ums next_state=%s", timeout_ms, LOG_STR_ARG(state_to_string_(next_state)));
  if (!this->radio_.start_rx(this->channel_)) {
    ESP_LOGW(TAG, "%s", failure_log);
    this->set_next_state_(State::IDLE);
    return false;
  }
  this->start_timeout_(timeout_ms);
  this->set_next_state_(next_state);
  return true;
}

void NartisWmbusComponent::handle_init_session_() {
  this->session_start_ms_ = millis();
  this->registry_.reset_all();
  this->retry_count_ = 0;

  ESP_LOGV(TAG, "handle_init_session_: associated=%s, sensor_count=%d",
           YESNO(this->associated_), (int) this->registry_.size());
  if (this->associated_) {
    ESP_LOGV(TAG, "handle_init_session_: already associated, skipping install+AARQ -> DATA_REQUEST");
    this->registry_.start_requests();
    this->set_next_state_(State::DATA_REQUEST);
  } else {
    ESP_LOGV(TAG, "handle_init_session_: not associated -> SEND_INSTALL");
    this->set_next_state_(State::SEND_INSTALL);
  }
}

void NartisWmbusComponent::handle_send_install_() {
  ESP_LOGD(TAG, "Sending SND-IR install request (pairing beacon)");

  if (!this->send_install_frame_()) {
    ESP_LOGW(TAG, "Failed to send install request");
    this->set_next_state_(State::IDLE);
    return;
  }

  this->retry_count_ = 0;
  this->start_rx_with_timeout_(INSTALL_TIMEOUT_MS, State::WAIT_INSTALL, "Failed to start RX for install reply");
}

void NartisWmbusComponent::handle_wait_install_() {
  if (this->check_timeout_()) {
    this->radio_.stop_rx();
    this->retry_count_++;
    if (this->retry_count_ >= MAX_RETRIES) {
      ESP_LOGW(TAG, "No install reply after %d retries — proceeding to AARQ anyway", MAX_RETRIES);
      this->set_next_state_(State::SEND_AARQ);
    } else {
      ESP_LOGD(TAG, "Install reply timeout, retry %d/%d", this->retry_count_, MAX_RETRIES);
      this->set_next_state_(State::SEND_INSTALL);
    }
    return;
  }

  uint8_t rf_buf[MAX_FRAME_SIZE];
  int16_t rf_len = this->radio_.check_rx(rf_buf, sizeof(rf_buf));
  if (rf_len == 0)
    return;
  ESP_LOGV(TAG, "handle_wait_install_: check_rx returned %d", rf_len);
  this->radio_.stop_rx();

  if (rf_len < 0) {
    ESP_LOGV(TAG, "handle_wait_install_: check_rx error, re-entering RX");
    this->radio_.start_rx(this->channel_);
    return;
  }

  uint8_t stripped[MAX_APDU_SIZE + 30];
  uint16_t stripped_len = wmbus_frame_parse(TAG, rf_buf, rf_len, stripped);
  if (stripped_len >= 11) {
    uint16_t m_field = stripped[2] | (stripped[3] << 8);
    uint32_t serial = stripped[4] | (stripped[5] << 8) | (stripped[6] << 16) | (stripped[7] << 24);
    char mfr[4];
    decode_manufacturer(m_field, mfr);
    ESP_LOGI(TAG, "Install reply: C=0x%02X M=%s S=%08X v=%d t=0x%02X CI=0x%02X", stripped[1], mfr, serial, stripped[8],
             stripped[9], stripped[10]);
    ESP_LOGV(TAG, "handle_wait_install_: install reply parsed OK, stripped_len=%d", stripped_len);
  } else {
    ESP_LOGD(TAG, "Install reply: bad frame (%d raw bytes, stripped=%d)", rf_len, stripped_len);
    this->radio_.start_rx(this->channel_);
    return;
  }

  ESP_LOGD(TAG, "Install exchange complete, proceeding to AARQ");
  this->set_next_state_(State::SEND_AARQ);
}

void NartisWmbusComponent::handle_send_aarq_() {
  bool can_encrypt = this->system_title_valid_;

  ESP_LOGV(TAG, "handle_send_aarq_: can_encrypt=%s system_title_valid=%s IC=%u",
           YESNO(can_encrypt), YESNO(this->system_title_valid_), this->invocation_counter_);
  if (can_encrypt) {
    ESP_LOGV(TAG, "handle_send_aarq_: meter sys_title=%02X%02X%02X%02X%02X%02X%02X%02X",
             this->meter_system_title_[0], this->meter_system_title_[1], this->meter_system_title_[2],
             this->meter_system_title_[3], this->meter_system_title_[4], this->meter_system_title_[5],
             this->meter_system_title_[6], this->meter_system_title_[7]);
    ESP_LOGD(TAG, "Sending AARQ (encrypted, SND-NR) — system title known");
    if (!this->transmit_dlms_(AARQ_TEMPLATE, AARQ_TEMPLATE_SIZE, WMBUS_C_SND_NR, true)) {
      ESP_LOGW(TAG, "Failed to send encrypted AARQ");
      this->set_next_state_(State::IDLE);
      return;
    }
  } else {
    ESP_LOGD(TAG, "Sending AARQ (unencrypted, SND-IR) — first association");
    if (!this->transmit_dlms_(AARQ_TEMPLATE, AARQ_TEMPLATE_SIZE, WMBUS_C_SND_IR, false)) {
      ESP_LOGW(TAG, "Failed to send AARQ");
      this->set_next_state_(State::IDLE);
      return;
    }
  }

  this->retry_count_ = 0;
  this->start_rx_with_timeout_(AARE_TIMEOUT_MS, State::WAIT_AARE, "Failed to start RX for AARE");
}

void NartisWmbusComponent::handle_wait_aare_() {
  if (this->check_timeout_()) {
    this->radio_.stop_rx();
    this->retry_count_++;
    if (this->retry_count_ >= MAX_RETRIES) {
      ESP_LOGW(TAG, "No AARE after %d retries", MAX_RETRIES);
      this->set_next_state_(State::IDLE);
    } else {
      ESP_LOGD(TAG, "AARE timeout, retry %d/%d", this->retry_count_, MAX_RETRIES);
      this->set_next_state_(State::SEND_AARQ);
    }
    return;
  }

  uint8_t rf_buf[MAX_FRAME_SIZE];
  int16_t rf_len = this->radio_.check_rx(rf_buf, sizeof(rf_buf));
  if (rf_len == 0)
    return;
  ESP_LOGV(TAG, "handle_wait_aare_: check_rx returned %d", rf_len);
  this->radio_.stop_rx();

  if (rf_len < 0) {
    ESP_LOGV(TAG, "handle_wait_aare_: check_rx error, re-entering RX");
    this->radio_.start_rx(this->channel_);
    return;
  }

  uint16_t len = this->process_rx_frame_(rf_buf, rf_len, this->apdu_buf_);
  if (len == 0) {
    ESP_LOGV(TAG, "handle_wait_aare_: process_rx_frame_ returned 0, re-entering RX");
    this->radio_.start_rx(this->channel_);
    return;
  }

  ESP_LOGV(TAG, "handle_wait_aare_: got DLMS payload %d bytes, tag=0x%02X", len, this->apdu_buf_[0]);
  if (!dlms_parse_aare(TAG, this->apdu_buf_, len, this->meter_system_title_, this->system_title_valid_)) {
    ESP_LOGW(TAG, "AARE parse failed (len=%d, first bytes: %02X %02X %02X %02X)",
             len, len > 0 ? this->apdu_buf_[0] : 0, len > 1 ? this->apdu_buf_[1] : 0,
             len > 2 ? this->apdu_buf_[2] : 0, len > 3 ? this->apdu_buf_[3] : 0);
    this->set_next_state_(State::IDLE);
    return;
  }

  ESP_LOGV(TAG, "handle_wait_aare_: AARE accepted, sys_title_valid=%s", YESNO(this->system_title_valid_));
  this->associated_ = true;
  this->registry_.start_requests();
  this->set_next_state_(State::DATA_REQUEST);
}

void NartisWmbusComponent::handle_data_request_() {
  if (!this->registry_.has_current_request()) {
    this->set_next_state_(State::PUBLISH);
    return;
  }

  NartisWmbusSensorBase *sensor = this->registry_.current_sensor();
  const char *current_obis = sensor->get_obis_code().c_str();

  uint8_t obis_bytes[6];
  sensor->parse_obis_bytes(obis_bytes);

  uint8_t get_req[20];
  uint16_t req_len = dlms_build_get_request(obis_bytes, sensor->get_class_id(), sensor->get_attribute(), get_req);

  ESP_LOGD(TAG, "GET.request for OBIS %s (class=%d, attr=%d)", current_obis, sensor->get_class_id(),
           sensor->get_attribute());

  if (!this->transmit_dlms_(get_req, req_len, WMBUS_C_SND_NR, true)) {
    ESP_LOGW(TAG, "Failed to send GET.request");
    sensor->record_failure();
    this->set_next_state_(State::DATA_NEXT);
    return;
  }

  this->retry_count_ = 0;
  if (!this->start_rx_with_timeout_(RESPONSE_TIMEOUT_MS, State::WAIT_RESPONSE, "Failed to start RX for response")) {
    sensor->record_failure();
    this->set_next_state_(State::DATA_NEXT);
  }
}

void NartisWmbusComponent::handle_wait_response_() {
  if (this->check_timeout_()) {
    this->radio_.stop_rx();
    this->retry_count_++;
    if (this->retry_count_ >= MAX_RETRIES) {
      ESP_LOGW(TAG, "No response for OBIS %s after %d retries — meter not responding, possibly displaced",
               this->registry_.current_sensor()->get_obis_code().c_str(), MAX_RETRIES);
      this->registry_.mark_current_failure();
      this->associated_ = false;
      this->set_next_state_(State::PUBLISH);
    } else {
      ESP_LOGD(TAG, "Response timeout, retry %d/%d", this->retry_count_, MAX_RETRIES);
      this->set_next_state_(State::DATA_REQUEST);
    }
    return;
  }

  uint8_t rf_buf[MAX_FRAME_SIZE];
  int16_t rf_len = this->radio_.check_rx(rf_buf, sizeof(rf_buf));
  if (rf_len == 0)
    return;
  ESP_LOGV(TAG, "handle_wait_response_: check_rx returned %d for OBIS %s", rf_len,
           this->registry_.current_sensor()->get_obis_code().c_str());
  this->radio_.stop_rx();

  if (rf_len < 0) {
    ESP_LOGV(TAG, "handle_wait_response_: check_rx error, re-entering RX");
    this->radio_.start_rx(this->channel_);
    return;
  }

  uint16_t len = this->process_rx_frame_(rf_buf, rf_len, this->apdu_buf_);
  if (len == 0) {
    ESP_LOGV(TAG, "handle_wait_response_: process_rx_frame_ returned 0, re-entering RX");
    this->radio_.start_rx(this->channel_);
    return;
  }

  ESP_LOGV(TAG, "handle_wait_response_: DLMS payload %d bytes, tag=0x%02X", len, this->apdu_buf_[0]);
  float value = 0.0f;
  char text_value[512];
  text_value[0] = '\0';
  bool is_text = false;
  if (dlms_parse_get_response(TAG, this->apdu_buf_, len, value, text_value, sizeof(text_value), is_text)) {
    if (!this->associated_) {
      ESP_LOGI(TAG, "Meter responding again — association restored");
      this->associated_ = true;
    }
    if (is_text) {
      ESP_LOGD(TAG, "OBIS %s = \"%s\"", this->registry_.current_sensor()->get_obis_code().c_str(), text_value);
      this->registry_.apply_current_text(text_value);
    } else {
      ESP_LOGD(TAG, "OBIS %s = %.3f", this->registry_.current_sensor()->get_obis_code().c_str(), value);
      this->registry_.apply_current_value(value);
    }
  } else {
    ESP_LOGW(TAG, "Failed to parse GET.response for OBIS %s (len=%d, bytes: %02X %02X %02X %02X)",
             this->registry_.current_sensor()->get_obis_code().c_str(), len,
             len > 0 ? this->apdu_buf_[0] : 0, len > 1 ? this->apdu_buf_[1] : 0,
             len > 2 ? this->apdu_buf_[2] : 0, len > 3 ? this->apdu_buf_[3] : 0);
    this->registry_.mark_current_failure();
  }

  this->set_next_state_(State::DATA_NEXT);
}

void NartisWmbusComponent::handle_data_next_() {
  this->registry_.advance_request();

  if (!this->registry_.has_current_request()) {
    this->set_next_state_(State::PUBLISH);
  } else {
    this->set_next_state_(State::DATA_REQUEST);
  }
}

void NartisWmbusComponent::handle_publish_() {
  ESP_LOGD(TAG, "Publishing sensor values");
  this->registry_.publish_ready();
  this->set_next_state_(State::IDLE);
}

// ============================================================================
// Main FSM Loop
// ============================================================================

void NartisWmbusComponent::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED) {
    // Only log this occasionally to avoid flooding
    static uint32_t last_not_ready_log = 0;
    if (millis() - last_not_ready_log > 5000) {
      ESP_LOGVV(TAG, "loop: not ready yet (is_ready=%s state=%s)",
                YESNO(this->is_ready()), LOG_STR_ARG(state_to_string_(this->state_)));
      last_not_ready_log = millis();
    }
    return;
  }

  if (this->state_ != State::IDLE && this->state_ != State::SNIFFING && this->state_ != State::LISTENING &&
      this->state_ != State::NOT_INITIALIZED && this->state_ != State::INIT_SESSION && this->check_session_timeout_()) {
    ESP_LOGW(TAG, "Session timeout (%ums) in state %s — aborting", SESSION_TIMEOUT_MS,
             LOG_STR_ARG(state_to_string_(this->state_)));
    this->abort_session_to_idle_();
    return;
  }

  switch (this->state_) {
    case State::IDLE:
      break;
    case State::INIT_SESSION:
      this->handle_init_session_();
      break;
    case State::SEND_INSTALL:
      this->handle_send_install_();
      break;
    case State::WAIT_INSTALL:
      this->handle_wait_install_();
      break;
    case State::SEND_AARQ:
      this->handle_send_aarq_();
      break;
    case State::WAIT_AARE:
      this->handle_wait_aare_();
      break;
    case State::DATA_REQUEST:
      this->handle_data_request_();
      break;
    case State::WAIT_RESPONSE:
      this->handle_wait_response_();
      break;
    case State::DATA_NEXT:
      this->handle_data_next_();
      break;
    case State::PUBLISH:
      this->handle_publish_();
      break;
    case State::LISTENING:
      this->listen_loop_();
      break;
    case State::SNIFFING:
      this->sniff_loop_();
      break;
    default:
      this->set_next_state_(State::IDLE);
      break;
  }
}

// ============================================================================
// Sniffer Mode
// ============================================================================

void NartisWmbusComponent::hex_to_str_(const uint8_t *data, uint16_t len, char *out, uint16_t out_size) {
  static const char hex[] = "0123456789ABCDEF";
  uint16_t pos = 0;
  for (uint16_t i = 0; i < len && pos + 2 < out_size; i++) {
    out[pos++] = hex[data[i] >> 4];
    out[pos++] = hex[data[i] & 0x0F];
  }
  out[pos] = '\0';
}

const LogString *NartisWmbusComponent::c_field_to_string_(uint8_t c_field) {
  switch (c_field) {
    case WMBUS_C_SND_NR:
      return LOG_STR("SND-NR");
    case WMBUS_C_SND_IR:
      return LOG_STR("SND-IR");
    case WMBUS_C_RSP_UD:
      return LOG_STR("RSP-UD");
    case WMBUS_C_SND_NKE:
      return LOG_STR("SND-NKE");
    case WMBUS_C_SND_UD:
      return LOG_STR("SND-UD");
    default:
      return LOG_STR("???");
  }
}

const LogString *NartisWmbusComponent::ci_field_to_string_(uint8_t ci_field) {
  switch (ci_field) {
    case WMBUS_CI_TPL_SHORT:
      return LOG_STR("TPL-SHORT");
    case WMBUS_CI_ENC:
      return LOG_STR("ENC-DLMS");
    case WMBUS_CI_PLAIN:
      return LOG_STR("PLAIN");
    case WMBUS_CI_RSP_UD_12B:
      return LOG_STR("RSP-UD/12B");
    case WMBUS_CI_RSP_UD_0B:
      return LOG_STR("RSP-UD/0B");
    default:
      return LOG_STR("???");
  }
}


void NartisWmbusComponent::log_raw_frame_(const uint8_t *data, uint16_t len) {
  // Log raw hex in chunks (ESP_LOG has ~256 char limit)
  char hex_buf[129];  // 64 bytes per line
  uint16_t pos = 0;
  ESP_LOGD(TAG, "SNIFF raw [%d bytes]:", len);
  while (pos < len) {
    uint16_t chunk = len - pos;
    if (chunk > 64)
      chunk = 64;
    hex_to_str_(&data[pos], chunk, hex_buf, sizeof(hex_buf));
    ESP_LOGD(TAG, "  %s", hex_buf);
    pos += chunk;
  }
}

void NartisWmbusComponent::log_parsed_frame_(const uint8_t *frame, uint16_t len) {
  if (len < 11) {
    ESP_LOGW(TAG, "SNIFF frame too short to parse (%d bytes)", len);
    return;
  }

  uint8_t l_field = frame[0];
  uint8_t c_field = frame[1];
  uint16_t m_field = frame[2] | (frame[3] << 8);
  // A-field: serial[4] + version[1] + type[1]
  uint32_t serial = frame[4] | (frame[5] << 8) | (frame[6] << 16) | (frame[7] << 24);
  uint8_t version = frame[8];
  uint8_t dev_type = frame[9];
  uint8_t ci_field = frame[10];

  char mfr[4];
  decode_manufacturer(m_field, mfr);

  ESP_LOGI(TAG, "SNIFF #%u: L=%d C=0x%02X(%s) M=%s(0x%04X) A=%08X v=%d t=0x%02X CI=0x%02X(%s)",
           this->sniffer_packet_count_, l_field, c_field, LOG_STR_ARG(c_field_to_string_(c_field)), mfr, m_field,
           serial, version, dev_type, ci_field, LOG_STR_ARG(ci_field_to_string_(ci_field)));

  // Parse payload depending on CI
  // CI=0x7A: TPL short header (4 bytes) + data
  // Other CI: data directly after CI
  uint16_t data_offset = 11;  // after DLL header (L+C+M+A+CI)
  uint16_t tpl_hdr_len = 0;

  if (ci_field == WMBUS_CI_TPL_SHORT && len >= 15) {
    // TPL short header: C_copy + CI_copy + access_nr + status
    tpl_hdr_len = 4;
    data_offset = 15;
    ESP_LOGI(TAG, "  TPL: C=0x%02X CI=0x%02X acc=%d status=0x%02X", frame[11], frame[12], frame[13], frame[14]);
  }

  if (data_offset >= len)
    return;

  const uint8_t *data = &frame[data_offset];
  uint16_t data_len = len - data_offset;

  // Check if encrypted: first byte is SC=0x94
  if (data_len >= 7 && (data[0] & DLMS_SC_ENCRYPTED)) {
    // Encrypted: SC(1) + pad(1) + IC(4) + ciphertext
    uint8_t sc = data[0];
    uint32_t ic = ((uint32_t) data[2] << 24) | ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 8) | data[5];
    uint16_t ct_len = data_len - 6;

    ESP_LOGI(TAG, "  ENC: SC=0x%02X IC=0x%08X(%u) ciphertext=%d bytes", sc, ic, ic, ct_len);

    // Try decryption if we have the key and a system title
    if (ct_len > 0 && ct_len < MAX_APDU_SIZE) {
      uint8_t nonce[12];
      bool can_decrypt = false;
      if (this->system_title_valid_) {
        memcpy(nonce, this->meter_system_title_, 8);
        can_decrypt = true;
      }
      if (can_decrypt) {
        nonce[8] = (ic >> 24) & 0xFF;
        nonce[9] = (ic >> 16) & 0xFF;
        nonce[10] = (ic >> 8) & 0xFF;
        nonce[11] = ic & 0xFF;

        uint8_t plain[MAX_APDU_SIZE];
        uint8_t sc_aad = data[0];  // SC byte as AAD
        if (dlms_aes_gcm_crypt(TAG, false, this->decryption_key_.data(), nonce, &sc_aad, 1, &data[6], ct_len, plain)) {
          char plain_hex[129];
          uint16_t dump_len = ct_len > 64 ? 64 : ct_len;
          hex_to_str_(plain, dump_len, plain_hex, sizeof(plain_hex));
          ESP_LOGI(TAG, "  DECRYPTED [%d]: %s%s", ct_len, plain_hex, ct_len > 64 ? "..." : "");
        } else {
          ESP_LOGD(TAG, "  Decryption failed (wrong key or system title?)");
        }
      } else {
        ESP_LOGD(TAG, "  No system title — cannot decrypt (need AARE first)");
      }
    }
  } else if (data_len > 0) {
    // Plaintext data
    char pay_hex[129];
    uint16_t dump_len = data_len > 64 ? 64 : data_len;
    hex_to_str_(data, dump_len, pay_hex, sizeof(pay_hex));
    ESP_LOGI(TAG, "  PLAIN [%d]: %s%s", data_len, pay_hex, data_len > 64 ? "..." : "");

    // Check if this is an AARE — extract system title for future decryption
    if (data_len > 2 && data[0] == DLMS_TAG_AARE) {
      ESP_LOGI(TAG, "  Detected AARE — attempting system title extraction");
      if (dlms_parse_aare(TAG, data, data_len, this->meter_system_title_, this->system_title_valid_)) {
        ESP_LOGI(TAG, "  System title captured: %02X%02X%02X%02X%02X%02X%02X%02X", this->meter_system_title_[0],
                 this->meter_system_title_[1], this->meter_system_title_[2], this->meter_system_title_[3],
                 this->meter_system_title_[4], this->meter_system_title_[5], this->meter_system_title_[6],
                 this->meter_system_title_[7]);
      }
    }
  }
}

void NartisWmbusComponent::sniff_loop_() {
  // Non-blocking RX check — radio stays in RX continuously (started in setup)
  uint8_t rf_buf[MAX_FRAME_SIZE];
  int16_t rf_len = this->radio_.check_rx(rf_buf, sizeof(rf_buf));
  if (rf_len == 0)
    return;  // nothing yet — instant return

  if (rf_len < 0) {
    // RX error — radio re-enters RX automatically in check_rx_polling_
    ESP_LOGW(TAG, "SNIFF: RX error");
    return;
  }

  this->sniffer_packet_count_++;

  // Log raw frame
  this->log_raw_frame_(rf_buf, rf_len);

  // Try to strip CRCs and parse
  uint8_t stripped[MAX_APDU_SIZE + 20];
  uint16_t stripped_len = wmbus_frame_parse(TAG, rf_buf, rf_len, stripped);

  if (stripped_len > 0) {
    ESP_LOGI(TAG, "SNIFF CRC: OK (raw=%d, clean=%d)", rf_len, stripped_len);
    this->log_parsed_frame_(stripped, stripped_len);
  } else {
    ESP_LOGW(TAG, "SNIFF CRC: FAILED (raw=%d bytes) — not a valid W-MBus frame", rf_len);
  }

  // Re-enter RX for next packet (check_rx leaves radio in standby after packet)
  this->radio_.start_rx(this->channel_);
}

// ============================================================================
// Listen Mode
// ============================================================================

const LogString *NartisWmbusComponent::mode_to_string_(Mode mode) {
  switch (mode) {
    case Mode::SESSION:
      return LOG_STR("SESSION");
    case Mode::LISTEN:
      return LOG_STR("LISTEN");
    case Mode::SNIFFER:
      return LOG_STR("SNIFFER");
    default:
      return LOG_STR("UNKNOWN");
  }
}

void NartisWmbusComponent::listen_loop_() {
  // Non-blocking RX check — radio stays in RX continuously (started in setup)
  uint8_t rf_buf[MAX_FRAME_SIZE];
  int16_t rf_len = this->radio_.check_rx(rf_buf, sizeof(rf_buf));
  if (rf_len == 0)
    return;  // nothing yet — instant return

  if (rf_len < 0) {
    ESP_LOGW(TAG, "LISTEN: RX error");
    return;
  }

  this->listen_packet_count_++;

  // Strip CRCs and log header
  uint8_t stripped[MAX_FRAME_SIZE];
  uint16_t stripped_len = wmbus_frame_parse(TAG, rf_buf, rf_len, stripped);
  if (stripped_len < 11) {
    ESP_LOGD(TAG, "LISTEN #%u: bad frame (%d raw bytes)", this->listen_packet_count_, rf_len);
    this->radio_.start_rx(this->channel_);
    return;
  }

  // Log W-MBus header
  uint16_t m_field = stripped[2] | (stripped[3] << 8);
  uint32_t serial = stripped[4] | (stripped[5] << 8) | (stripped[6] << 16) | (stripped[7] << 24);
  char mfr[4];
  decode_manufacturer(m_field, mfr);
  ESP_LOGI(TAG, "LISTEN #%u: C=0x%02X(%s) M=%s S=%08X v=%d t=0x%02X CI=0x%02X(%s)", this->listen_packet_count_,
           stripped[1], LOG_STR_ARG(c_field_to_string_(stripped[1])), mfr, serial, stripped[8], stripped[9],
           stripped[10], LOG_STR_ARG(ci_field_to_string_(stripped[10])));

  // Extract DLMS APDU (re-parses CRCs internally — minor overhead)
  uint16_t dlms_len = this->process_rx_frame_(rf_buf, rf_len, this->apdu_buf_);
  if (dlms_len == 0) {
    ESP_LOGD(TAG, "LISTEN: no DLMS payload");
    this->radio_.start_rx(this->channel_);
    return;
  }

  // Log DLMS APDU hex
  char hex[129];
  uint16_t dump = dlms_len > 64 ? 64 : dlms_len;
  hex_to_str_(this->apdu_buf_, dump, hex, sizeof(hex));
  ESP_LOGI(TAG, "LISTEN: DLMS [%d]: %s%s", dlms_len, hex, dlms_len > 64 ? "..." : "");

  // Parse and extract values
  this->listen_parse_dlms_(this->apdu_buf_, dlms_len);

  // Re-enter RX for next packet
  this->radio_.start_rx(this->channel_);
}

void NartisWmbusComponent::listen_parse_dlms_(const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;

  // Log known APDU tags for context
  uint8_t tag = data[0];
  switch (tag) {
    case DLMS_TAG_DATA_NOTIFICATION:
      ESP_LOGI(TAG, "LISTEN: data-notification");
      break;
    case DLMS_TAG_GET_RESPONSE:
      ESP_LOGI(TAG, "LISTEN: GET.response");
      break;
    case DLMS_TAG_SET_RESPONSE:
      ESP_LOGI(TAG, "LISTEN: SET.response");
      break;
    case DLMS_TAG_ACTION_RESPONSE:
      ESP_LOGI(TAG, "LISTEN: ACTION.response");
      break;
    case DLMS_TAG_AARE: {
      ESP_LOGI(TAG, "LISTEN: AARE — extracting system title");
      dlms_parse_aare(TAG, data, len, this->meter_system_title_, this->system_title_valid_);
      return;
    }
    default:
      ESP_LOGI(TAG, "LISTEN: DLMS tag 0x%02X", tag);
      break;
  }

  // TODO: parse push data structure once format is known
  // For now, full hex dump is logged by listen_loop_
}

void NartisWmbusComponent::scan_obis_values_(const uint8_t * /*data*/, uint16_t /*len*/) {
  // TODO: implement OBIS+value extraction from push data once format is known
}

}  // namespace esphome::nartis_wmbus
