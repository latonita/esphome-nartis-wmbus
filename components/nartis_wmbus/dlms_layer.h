#pragma once

#include <array>
#include <cstdint>

namespace esphome::nartis_wmbus {

// --- DLMS Security Control byte (IEC 62056-5-3, xDLMS) ---
static constexpr uint8_t DLMS_SC_ENCRYPTED = 0x10;     // bit 4: ciphertext present
static constexpr uint8_t DLMS_SC_AUTHENTICATED = 0x20;  // bit 5: auth tag present
static constexpr uint8_t DLMS_SC_COMPRESSED = 0x80;     // bit 7: payload is DEFLATE compressed
static constexpr uint8_t DLMS_SC_SUITE_MASK = 0x07;     // bits 0-2: security suite ID
static constexpr uint8_t DLMS_SC_SUITE_0 = 0x04;        // Security Suite 0 (AES-GCM-128)
// Common combined SC values seen on air:
static constexpr uint8_t DLMS_SC_ENC_COMP = 0x94;       // encrypted + compressed + Suite 0

// --- DLMS data types ---
static constexpr uint8_t DLMS_TYPE_BOOLEAN = 0x03;
static constexpr uint8_t DLMS_TYPE_INT8 = 0x0F;
static constexpr uint8_t DLMS_TYPE_UINT8 = 0x11;
static constexpr uint8_t DLMS_TYPE_INT16 = 0x10;
static constexpr uint8_t DLMS_TYPE_UINT16 = 0x12;
static constexpr uint8_t DLMS_TYPE_INT32 = 0x05;
static constexpr uint8_t DLMS_TYPE_UINT32 = 0x06;
static constexpr uint8_t DLMS_TYPE_INT64 = 0x14;
static constexpr uint8_t DLMS_TYPE_UINT64 = 0x15;
static constexpr uint8_t DLMS_TYPE_FLOAT32 = 0x17;
static constexpr uint8_t DLMS_TYPE_FLOAT64 = 0x18;
static constexpr uint8_t DLMS_TYPE_ENUM = 0x16;
static constexpr uint8_t DLMS_TYPE_OCTET_STRING = 0x09;
static constexpr uint8_t DLMS_TYPE_VISIBLE_STRING = 0x0A;
static constexpr uint8_t DLMS_TYPE_UTF8_STRING = 0x0C;
static constexpr uint8_t DLMS_TYPE_DATETIME = 0x19;

static constexpr uint8_t DLMS_TAG_DATA_NOTIFICATION = 0x0C;
static constexpr uint8_t DLMS_TAG_GET_RESPONSE = 0xC4;
static constexpr uint8_t DLMS_TAG_SET_RESPONSE = 0xC5;
static constexpr uint8_t DLMS_TAG_ACTION_RESPONSE = 0xC7;
static constexpr uint8_t DLMS_TAG_AARE = 0x61;
static constexpr uint8_t DLMS_TAG_AARE_RESULT = 0xA2;
static constexpr uint8_t DLMS_TAG_AARE_RESPONDING_AP_TITLE = 0xA4;

static constexpr uint8_t ASN1_TAG_INTEGER = 0x02;
static constexpr uint8_t ASN1_TAG_OCTET_STRING = 0x04;
static constexpr uint8_t ASN1_INTEGER_U8_LEN = 0x01;

static constexpr uint8_t DLMS_ASSOCIATION_RESULT_ACCEPTED = 0x00;
static constexpr uint8_t DLMS_SYSTEM_TITLE_SIZE = 8;
static constexpr uint8_t DLMS_AARE_RESULT_FIELD_MIN_LEN = 3;
static constexpr uint8_t DLMS_AARE_AP_TITLE_FIELD_LEN = 10;

static constexpr uint8_t DLMS_CLIENT_SAP = 0x20;
static constexpr uint16_t DLMS_SERVER_SAP = 0x0001;
static constexpr char DLMS_PASSWORD[] = "123456";

extern const uint8_t AARQ_TEMPLATE[];
extern const uint16_t AARQ_TEMPLATE_SIZE;

// --- PDU builders and parsers ---
uint16_t dlms_build_get_request(const uint8_t obis[6], uint16_t class_id, uint8_t attr, uint8_t *out);
bool dlms_parse_aare(const char *log_tag, const uint8_t *data, uint16_t len, uint8_t meter_system_title[8],
                     bool &system_title_valid);
bool dlms_parse_get_response(const char *log_tag, const uint8_t *data, uint16_t len, float &value, char *text_buf,
                             uint16_t text_buf_size, bool &is_text);

// --- DLMS Security (AES-128-GCM, IEC 62056-5-3) ---
bool dlms_aes_gcm_crypt(const char *log_tag, bool encrypt, const uint8_t *key, const uint8_t *nonce,
                        const uint8_t *aad, uint16_t aad_len, const uint8_t *input, uint16_t len, uint8_t *output,
                        uint8_t *tag_buf = nullptr, uint8_t tag_len = 0);
uint16_t dlms_encrypt(const char *log_tag, const std::array<uint8_t, 16> &key, const std::array<uint8_t, 8> &system_title,
                      uint32_t invocation_counter, const uint8_t *apdu, uint16_t apdu_len, uint8_t *out);
uint16_t dlms_decrypt(const char *log_tag, const std::array<uint8_t, 16> &key, const uint8_t meter_system_title[8],
                      const uint8_t *enc_payload, uint16_t enc_len, uint8_t *apdu_out);

}  // namespace esphome::nartis_wmbus
