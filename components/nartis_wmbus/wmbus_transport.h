#pragma once

#include <cstdint>

namespace esphome::nartis_wmbus {

// --- W-MBus Data Link Layer (EN 13757-4) ---

// C-field values
static constexpr uint8_t WMBUS_C_SND_NR = 0x44;       // Send, No Reply expected
static constexpr uint8_t WMBUS_C_SND_IR = 0x46;       // Send, Install Request
static constexpr uint8_t WMBUS_C_RSP_UD = 0x08;       // Response, User Data
static constexpr uint8_t WMBUS_C_SND_NKE = 0x00;      // Send, No Keep Alive
static constexpr uint8_t WMBUS_C_SND_UD = 0x40;       // Send, User Data

// CI-field values
static constexpr uint8_t WMBUS_CI_TPL_SHORT = 0x7A;   // TPL short header (4 bytes)
static constexpr uint8_t WMBUS_CI_ENC = 0xBD;         // Encrypted DLMS payload
static constexpr uint8_t WMBUS_CI_PLAIN = 0xC5;       // Unencrypted command
static constexpr uint8_t WMBUS_CI_RSP_UD_12B = 0x72;  // RSP_UD with 12-byte header
static constexpr uint8_t WMBUS_CI_RSP_UD_0B = 0x78;   // RSP_UD with 0-byte header

// Block sizes for CRC interleaving
static constexpr uint8_t WMBUS_BLOCK_SIZE = 16;       // Data bytes per CRC block
static constexpr uint8_t WMBUS_FIRST_BLOCK = 10;      // First block: L+C+M+A = 10 bytes

// Frame limits
static constexpr uint16_t MAX_FRAME_SIZE = 512;
static constexpr uint16_t MAX_APDU_SIZE = 300;
static constexpr uint16_t MAX_APDU_DECOMPRESSED = 1024;  // decompressed DLMS APDUs can be larger
static constexpr uint8_t INSTALL_PAYLOAD_SIZE = 13;

// Our W-MBus identity
static constexpr uint16_t OUR_MANUFACTURER = 0x3832;  // "NAR" (Nartis)
static constexpr uint8_t OUR_ADDRESS[4] = {0x00, 0x00, 0x01, 0x00};
static constexpr uint8_t OUR_VERSION = 0x01;
static constexpr uint8_t OUR_DEVICE_TYPE = 0x00;

// Transport functions
uint16_t wmbus_crc16_en13757(const uint8_t *data, uint16_t len);
uint16_t wmbus_frame_build(uint8_t c_field, const uint8_t *payload, uint16_t pay_len, uint8_t access_nr, uint8_t *out);
uint16_t wmbus_frame_parse(const char *log_tag, const uint8_t *frame, uint16_t frame_len, uint8_t *out);
void build_install_payload(const char *meter_id, uint8_t out[INSTALL_PAYLOAD_SIZE]);
void decode_manufacturer(uint16_t m_field, char out[4]);

}  // namespace esphome::nartis_wmbus
