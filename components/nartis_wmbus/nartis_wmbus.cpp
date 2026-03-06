#include "nartis_wmbus.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"

#include "mbedtls/esp_config.h"
#include "mbedtls/gcm.h"

#include <cstring>
#include <algorithm>

namespace esphome {
namespace nartis_wmbus {

static const char *const TAG = "nartis_wmbus";

// ============================================================================
// CRC-16/EN-13757 (W-MBus standard, poly 0x3D65, non-reflected, final ~CRC)
// Table-lookup, matches firmware at 0xB750. Table from ROM at 0xB77C.
// Check value: CRC("123456789") = 0xC2B7
// ============================================================================

static const uint16_t CRC16_EN13757_TABLE[256] = {
    0x0000, 0x3D65, 0x7ACA, 0x47AF, 0xF594, 0xC8F1, 0x8F5E, 0xB23B,
    0xD64D, 0xEB28, 0xAC87, 0x91E2, 0x23D9, 0x1EBC, 0x5913, 0x6476,
    0x91FF, 0xAC9A, 0xEB35, 0xD650, 0x646B, 0x590E, 0x1EA1, 0x23C4,
    0x47B2, 0x7AD7, 0x3D78, 0x001D, 0xB226, 0x8F43, 0xC8EC, 0xF589,
    0x1E9B, 0x23FE, 0x6451, 0x5934, 0xEB0F, 0xD66A, 0x91C5, 0xACA0,
    0xC8D6, 0xF5B3, 0xB21C, 0x8F79, 0x3D42, 0x0027, 0x4788, 0x7AED,
    0x8F64, 0xB201, 0xF5AE, 0xC8CB, 0x7AF0, 0x4795, 0x003A, 0x3D5F,
    0x5929, 0x644C, 0x23E3, 0x1E86, 0xACBD, 0x91D8, 0xD677, 0xEB12,
    0x3D36, 0x0053, 0x47FC, 0x7A99, 0xC8A2, 0xF5C7, 0xB268, 0x8F0D,
    0xEB7B, 0xD61E, 0x91B1, 0xACD4, 0x1EEF, 0x238A, 0x6425, 0x5940,
    0xACC9, 0x91AC, 0xD603, 0xEB66, 0x595D, 0x6438, 0x2397, 0x1EF2,
    0x7A84, 0x47E1, 0x004E, 0x3D2B, 0x8F10, 0xB275, 0xF5DA, 0xC8BF,
    0x23AD, 0x1EC8, 0x5967, 0x6402, 0xD639, 0xEB5C, 0xACF3, 0x9196,
    0xF5E0, 0xC885, 0x8F2A, 0xB24F, 0x0074, 0x3D11, 0x7ABE, 0x47DB,
    0xB252, 0x8F37, 0xC898, 0xF5FD, 0x47C6, 0x7AA3, 0x3D0C, 0x0069,
    0x641F, 0x597A, 0x1ED5, 0x23B0, 0x918B, 0xACEE, 0xEB41, 0xD624,
    0x7A6C, 0x4709, 0x00A6, 0x3DC3, 0x8FF8, 0xB29D, 0xF532, 0xC857,
    0xAC21, 0x9144, 0xD6EB, 0xEB8E, 0x59B5, 0x64D0, 0x237F, 0x1E1A,
    0xEB93, 0xD6F6, 0x9159, 0xAC3C, 0x1E07, 0x2362, 0x64CD, 0x59A8,
    0x3DDE, 0x00BB, 0x4714, 0x7A71, 0xC84A, 0xF52F, 0xB280, 0x8FE5,
    0x64F7, 0x5992, 0x1E3D, 0x2358, 0x9163, 0xAC06, 0xEBA9, 0xD6CC,
    0xB2BA, 0x8FDF, 0xC870, 0xF515, 0x472E, 0x7A4B, 0x3DE4, 0x0081,
    0xF508, 0xC86D, 0x8FC2, 0xB2A7, 0x009C, 0x3DF9, 0x7A56, 0x4733,
    0x2345, 0x1E20, 0x598F, 0x64EA, 0xD6D1, 0xEBB4, 0xAC1B, 0x917E,
    0x475A, 0x7A3F, 0x3D90, 0x00F5, 0xB2CE, 0x8FAB, 0xC804, 0xF561,
    0x9117, 0xAC72, 0xEBDD, 0xD6B8, 0x6483, 0x59E6, 0x1E49, 0x232C,
    0xD6A5, 0xEBC0, 0xAC6F, 0x910A, 0x2331, 0x1E54, 0x59FB, 0x649E,
    0x00E8, 0x3D8D, 0x7A22, 0x4747, 0xF57C, 0xC819, 0x8FB6, 0xB2D3,
    0x59C1, 0x64A4, 0x230B, 0x1E6E, 0xAC55, 0x9130, 0xD69F, 0xEBFA,
    0x8F8C, 0xB2E9, 0xF546, 0xC823, 0x7A18, 0x477D, 0x00D2, 0x3DB7,
    0xC83E, 0xF55B, 0xB2F4, 0x8F91, 0x3DAA, 0x00CF, 0x4760, 0x7A05,
    0x1E73, 0x2316, 0x64B9, 0x59DC, 0xEBE7, 0xD682, 0x912D, 0xAC48,
};

uint16_t NartisWmbusComponent::crc16_en13757(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0x0000;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t idx = (uint8_t)(crc >> 8) ^ data[i];
    crc = (crc << 8) ^ CRC16_EN13757_TABLE[idx];
  }
  return ~crc & 0xFFFF;
}

// ============================================================================
// W-MBus Frame Build (TX) — block interleaved CRC
// Per firmware frame_build (0xBB88):
//   DLL: L + C + M(2) + A(6) + CI(0x7A)
//   TPL short header: C_copy + CI_copy(0x7A) + access_nr + status(0x00)
//   Payload: [SC + pad + IC(4,BE) + ciphertext] or [plaintext APDU]
// ============================================================================

uint16_t NartisWmbusComponent::wmbus_frame_build_(uint8_t c_field, uint8_t ci_field,
                                                    const uint8_t *payload, uint16_t pay_len,
                                                    uint8_t *out) {
  uint8_t raw[MAX_APDU_SIZE + 30];
  uint16_t raw_len = 0;

  // L-field placeholder (filled below)
  raw[raw_len++] = 0;

  // C-field (DLL)
  raw[raw_len++] = c_field;

  // M-field (manufacturer, 2 bytes LE)
  raw[raw_len++] = OUR_MANUFACTURER & 0xFF;
  raw[raw_len++] = (OUR_MANUFACTURER >> 8) & 0xFF;

  // A-field (serial[4] + version + type)
  memcpy(&raw[raw_len], OUR_ADDRESS, 4);
  raw_len += 4;
  raw[raw_len++] = OUR_VERSION;
  raw[raw_len++] = OUR_DEVICE_TYPE;

  // CI-field — always 0x7A (TPL short header) per firmware
  raw[raw_len++] = 0x7A;

  // TPL short header (4 bytes) — firmware buf[0x0B..0x0E]
  raw[raw_len++] = c_field;      // C-field copy
  raw[raw_len++] = 0x7A;         // CI-field copy (always 0x7A)
  raw[raw_len++] = access_nr_;   // Access number
  raw[raw_len++] = 0x00;         // Status byte

  // Payload (encrypted SC+pad+IC+ciphertext, or plain APDU)
  memcpy(&raw[raw_len], payload, pay_len);
  raw_len += pay_len;

  // L-field = bytes after L = raw_len - 1
  raw[0] = raw_len - 1;

  // Insert CRC-16/DNP per block
  uint16_t out_len = 0;
  uint16_t pos = 0;

  // First block: 10 bytes
  uint16_t first_block = WMBUS_FIRST_BLOCK;
  if (first_block > raw_len)
    first_block = raw_len;

  memcpy(&out[out_len], &raw[pos], first_block);
  out_len += first_block;
  pos += first_block;

  uint16_t crc = crc16_en13757(&raw[0], first_block);
  out[out_len++] = (crc >> 8) & 0xFF;  // high byte first (BE)
  out[out_len++] = crc & 0xFF;

  // Remaining blocks: 16 bytes each
  while (pos < raw_len) {
    uint16_t block_len = raw_len - pos;
    if (block_len > WMBUS_BLOCK_SIZE)
      block_len = WMBUS_BLOCK_SIZE;

    memcpy(&out[out_len], &raw[pos], block_len);
    out_len += block_len;

    crc = crc16_en13757(&raw[pos], block_len);
    out[out_len++] = (crc >> 8) & 0xFF;  // high byte first (BE)
    out[out_len++] = crc & 0xFF;

    pos += block_len;
  }

  return out_len;
}

// ============================================================================
// W-MBus Frame Parse (RX) — strip block CRCs
// ============================================================================

uint16_t NartisWmbusComponent::wmbus_frame_parse_(const uint8_t *frame, uint16_t frame_len,
                                                    uint8_t *out) {
  uint16_t in_pos = 0;
  uint16_t out_len = 0;

  // First block: 10 data bytes + 2 CRC bytes
  if (in_pos + WMBUS_FIRST_BLOCK + 2 > frame_len)
    return 0;

  uint16_t crc_calc = crc16_en13757(&frame[in_pos], WMBUS_FIRST_BLOCK);
  uint16_t crc_recv = (frame[in_pos + WMBUS_FIRST_BLOCK] << 8) |  // high byte first (BE)
                      frame[in_pos + WMBUS_FIRST_BLOCK + 1];
  if (crc_calc != crc_recv) {
    ESP_LOGW(TAG, "CRC error in first block: calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
    return 0;
  }

  memcpy(&out[out_len], &frame[in_pos], WMBUS_FIRST_BLOCK);
  out_len += WMBUS_FIRST_BLOCK;
  in_pos += WMBUS_FIRST_BLOCK + 2;

  // Remaining blocks
  uint8_t l_field = out[0];
  uint16_t expected_data = l_field + 1;  // L doesn't count itself

  while (out_len < expected_data && in_pos < frame_len) {
    uint16_t remaining = expected_data - out_len;
    uint16_t block_len = (remaining > WMBUS_BLOCK_SIZE) ? WMBUS_BLOCK_SIZE : remaining;

    if (in_pos + block_len + 2 > frame_len)
      return 0;

    crc_calc = crc16_en13757(&frame[in_pos], block_len);
    crc_recv = (frame[in_pos + block_len] << 8) | frame[in_pos + block_len + 1];  // BE
    if (crc_calc != crc_recv) {
      ESP_LOGW(TAG, "CRC error in data block at pos %d", in_pos);
      return 0;
    }

    memcpy(&out[out_len], &frame[in_pos], block_len);
    out_len += block_len;
    in_pos += block_len + 2;
  }

  return out_len;
}

// ============================================================================
// AES-128-GCM using mbedtls (ESP32)
// ============================================================================

bool NartisWmbusComponent::aes_gcm_crypt_(bool encrypt, const uint8_t *key,
                                            const uint8_t *nonce,
                                            const uint8_t *aad, uint16_t aad_len,
                                            const uint8_t *input, uint16_t len,
                                            uint8_t *output,
                                            uint8_t *tag_buf, uint8_t tag_len) {
  mbedtls_gcm_context gcm_ctx;
  mbedtls_gcm_init(&gcm_ctx);

  int ret = mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "GCM setkey failed: %d", ret);
    mbedtls_gcm_free(&gcm_ctx);
    return false;
  }

  int mode = encrypt ? MBEDTLS_GCM_ENCRYPT : MBEDTLS_GCM_DECRYPT;
  ret = mbedtls_gcm_starts(&gcm_ctx, mode, nonce, 12);
  if (ret != 0) {
    ESP_LOGE(TAG, "GCM starts failed: %d", ret);
    mbedtls_gcm_free(&gcm_ctx);
    return false;
  }

  // Process AAD (Additional Authenticated Data)
  // Per firmware gcm_decrypt_service (0x1B300): AAD = SC byte only (1 byte)
  if (aad != nullptr && aad_len > 0) {
    ret = mbedtls_gcm_update_ad(&gcm_ctx, aad, aad_len);
    if (ret != 0) {
      ESP_LOGE(TAG, "GCM update_ad failed: %d", ret);
      mbedtls_gcm_free(&gcm_ctx);
      return false;
    }
  }

  size_t outlen = 0;
  ret = mbedtls_gcm_update(&gcm_ctx, input, len, output, len, &outlen);
  if (ret != 0) {
    ESP_LOGE(TAG, "GCM %s failed: %d", encrypt ? "encrypt" : "decrypt", ret);
    mbedtls_gcm_free(&gcm_ctx);
    return false;
  }

  // Finalize GCM: generate or verify auth tag
  uint8_t computed_tag[16];
  size_t finish_outlen = 0;
  ret = mbedtls_gcm_finish(&gcm_ctx, nullptr, 0, &finish_outlen, computed_tag, sizeof(computed_tag));
  mbedtls_gcm_free(&gcm_ctx);

  if (ret != 0) {
    ESP_LOGE(TAG, "GCM finish failed: %d", ret);
    return false;
  }

  if (tag_buf != nullptr && tag_len > 0) {
    if (encrypt) {
      // Output the generated tag
      memcpy(tag_buf, computed_tag, tag_len);
    } else {
      // Verify tag: constant-time compare to prevent timing attacks
      uint8_t diff = 0;
      for (uint8_t i = 0; i < tag_len; i++) {
        diff |= computed_tag[i] ^ tag_buf[i];
      }
      if (diff != 0) {
        ESP_LOGW(TAG, "GCM auth tag mismatch — frame may be corrupted or tampered");
        return false;
      }
    }
  }

  return true;
}

// ============================================================================
// W-MBus Encrypt (DLMS APDU -> encrypted payload)
// Per firmware frame_build (0xBB88):
//   Output: SC(1) + pad(1,0x00) + IC(4,BE) + ciphertext
//   Nonce: system_title[8] || IC[4]
// NOTE: TX nonce should use OUR (CIU) system title, not meter's (issue #4)
// ============================================================================

uint16_t NartisWmbusComponent::wmbus_encrypt_(const uint8_t *dlms_apdu, uint16_t apdu_len,
                                                uint8_t *out) {
  uint16_t pos = 0;

  // SC byte (Security Control: AES-GCM-128 encrypted)
  out[pos++] = WMBUS_SC_GCM;

  // Padding byte (firmware buf[0x10] = 0x00)
  out[pos++] = 0x00;

  // Invocation Counter (4 bytes, big-endian) — firmware buf[0x11..0x14]
  uint32_t ic = invocation_counter_;
  out[pos++] = (ic >> 24) & 0xFF;
  out[pos++] = (ic >> 16) & 0xFF;
  out[pos++] = (ic >> 8) & 0xFF;
  out[pos++] = ic & 0xFF;

  // Build GCM nonce: OUR_system_title[8] || IC[4]
  // Per firmware: TX encrypt uses CIU's own system title (RAM 0x20005ABC)
  //               RX decrypt uses meter's system title (RAM 0x20005924)
  uint8_t nonce[12];
  memcpy(nonce, system_title_.data(), 8);
  nonce[8] = (ic >> 24) & 0xFF;
  nonce[9] = (ic >> 16) & 0xFF;
  nonce[10] = (ic >> 8) & 0xFF;
  nonce[11] = ic & 0xFF;

  invocation_counter_++;

  // Encrypt DLMS APDU — firmware buf[0x15..] = ciphertext
  // AAD = SC byte only (1 byte) per firmware gcm_decrypt_service (0x1B300)
  uint8_t sc_aad = WMBUS_SC_GCM;
  if (!aes_gcm_crypt_(true, decryption_key_.data(), nonce, &sc_aad, 1, dlms_apdu, apdu_len, &out[pos])) {
    return 0;
  }
  pos += apdu_len;

  return pos;
}

// ============================================================================
// W-MBus Decrypt (encrypted payload -> DLMS APDU)
// Per firmware frame_parse (0xBF5E):
//   Input: SC(1) + pad(1) + IC(4,BE) + ciphertext
//   Nonce: meter_system_title[8] || IC[4]
// ============================================================================

uint16_t NartisWmbusComponent::wmbus_decrypt_(const uint8_t *enc_payload, uint16_t enc_len,
                                                uint8_t *dlms_out) {
  // Minimum: SC(1) + pad(1) + IC(4) + at least 1 byte ciphertext
  if (enc_len < 7) {
    ESP_LOGW(TAG, "Encrypted payload too short: %d", enc_len);
    return 0;
  }

  uint16_t pos = 0;

  // SC byte (Security Control)
  uint8_t sc = enc_payload[pos++];

  // Padding byte (skip)
  pos++;

  if ((sc & 0x10) == 0) {
    // Not encrypted — return remaining payload as-is
    uint16_t plain_len = enc_len - pos;
    memcpy(dlms_out, &enc_payload[pos], plain_len);
    return plain_len;
  }

  // Invocation Counter (4 bytes, big-endian) — firmware buf[0x0F..0x12]
  uint32_t ic = ((uint32_t) enc_payload[pos] << 24) |
                ((uint32_t) enc_payload[pos + 1] << 16) |
                ((uint32_t) enc_payload[pos + 2] << 8) |
                enc_payload[pos + 3];
  pos += 4;

  // Build GCM nonce: meter_system_title[8] || IC[4]
  uint8_t nonce[12];
  memcpy(nonce, meter_system_title_, 8);
  nonce[8] = (ic >> 24) & 0xFF;
  nonce[9] = (ic >> 16) & 0xFF;
  nonce[10] = (ic >> 8) & 0xFF;
  nonce[11] = ic & 0xFF;

  uint16_t cipher_len = enc_len - pos;
  uint8_t *tag_ptr = nullptr;
  uint8_t tag_len = 0;

  // If SC has auth tag bit (0x20), last 12 bytes are GCM tag
  if (sc & 0x20) {
    if (cipher_len < 12)
      return 0;
    cipher_len -= 12;
    tag_ptr = const_cast<uint8_t *>(&enc_payload[pos + cipher_len]);
    tag_len = 12;
  }

  // AAD = SC byte only (1 byte) per firmware gcm_decrypt_service (0x1B300)
  if (!aes_gcm_crypt_(false, decryption_key_.data(), nonce, &sc, 1,
                       &enc_payload[pos], cipher_len, dlms_out, tag_ptr, tag_len)) {
    return 0;
  }

  ESP_LOGD(TAG, "Decrypted %d bytes, IC=0x%08X", cipher_len, ic);
  return cipher_len;
}

// ============================================================================
// DLMS APDU Builders
// ============================================================================

uint16_t NartisWmbusComponent::build_get_request_(const uint8_t obis[6], uint16_t class_id,
                                                    uint8_t attr, uint8_t *out) {
  uint16_t pos = 0;

  // GET.request PDU
  out[pos++] = 0xC0;  // GetRequest tag
  out[pos++] = 0x01;  // GetRequestNormal
  out[pos++] = 0xC0;  // invoke-id-and-priority (high priority)

  // Cosem-Attribute-Descriptor
  out[pos++] = (class_id >> 8) & 0xFF;
  out[pos++] = class_id & 0xFF;

  // OBIS code (6 bytes)
  memcpy(&out[pos], obis, 6);
  pos += 6;

  // Attribute index
  out[pos++] = attr;

  // Access selection (none)
  out[pos++] = 0x00;

  return pos;
}

bool NartisWmbusComponent::parse_aare_(const uint8_t *data, uint16_t len) {
  // AARE tag = 0x61
  if (len < 2 || data[0] != 0x61) {
    ESP_LOGW(TAG, "Not an AARE (tag=0x%02X)", data[0]);
    return false;
  }

  // Search for result (tag 0xA2)
  // Search for responding-AP-title (tag 0xA4) which contains system title
  bool result_ok = false;
  bool found_system_title = false;
  uint16_t pos = 2;  // skip tag + length

  while (pos < len) {
    uint8_t tag = data[pos];
    if (pos + 1 >= len) break;
    uint8_t tag_len = data[pos + 1];
    pos += 2;

    if (pos + tag_len > len) break;

    if (tag == 0xA2) {
      // Association result
      // Should contain result = 0 (accepted)
      // Format: A2 03 02 01 00 (integer, 1 byte, value 0)
      if (tag_len >= 3 && data[pos] == 0x02 && data[pos + 1] == 0x01) {
        uint8_t result = data[pos + 2];
        if (result == 0) {
          result_ok = true;
          ESP_LOGD(TAG, "AARE: association accepted");
        } else {
          ESP_LOGW(TAG, "AARE: association rejected (result=%d)", result);
          return false;
        }
      }
    } else if (tag == 0xA4) {
      // responding-AP-title — contains meter's system title
      // Format: A4 0A 04 08 <8 bytes system title>
      if (tag_len >= 10 && data[pos] == 0x04 && data[pos + 1] == 0x08) {
        memcpy(meter_system_title_, &data[pos + 2], 8);
        system_title_valid_ = true;
        found_system_title = true;
        ESP_LOGD(TAG, "AARE: meter system title: %02X%02X%02X%02X%02X%02X%02X%02X",
                 meter_system_title_[0], meter_system_title_[1],
                 meter_system_title_[2], meter_system_title_[3],
                 meter_system_title_[4], meter_system_title_[5],
                 meter_system_title_[6], meter_system_title_[7]);
      }
    }

    pos += tag_len;
  }

  if (!result_ok) {
    ESP_LOGW(TAG, "AARE: no result field found");
    return false;
  }
  if (!found_system_title) {
    ESP_LOGW(TAG, "AARE: no system title found (will try without)");
  }

  return true;
}

// Decode a single DLMS A-XDR data element (type_tag + value bytes).
// Returns true on success. Sets is_text=true and fills text_value for string types,
// otherwise fills numeric value.
static bool dlms_decode_data_element(uint8_t type_tag, const uint8_t *data, uint16_t avail,
                                     float &value, std::string &text_value, bool &is_text) {
  is_text = false;

  auto be16 = [](const uint8_t *p) -> uint16_t { return (uint16_t)((p[0] << 8) | p[1]); };
  auto be32 = [](const uint8_t *p) -> uint32_t {
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) |
           ((uint32_t) p[2] << 8) | (uint32_t) p[3];
  };

  switch (type_tag) {
    // --- Numeric types (fixed length) ---
    case DLMS_TYPE_UINT8:
      if (avail >= 1) { value = static_cast<float>(data[0]); return true; }
      break;
    case DLMS_TYPE_INT8:
      if (avail >= 1) { value = static_cast<float>(static_cast<int8_t>(data[0])); return true; }
      break;
    case DLMS_TYPE_UINT16:
      if (avail >= 2) { value = static_cast<float>(be16(data)); return true; }
      break;
    case DLMS_TYPE_INT16:
      if (avail >= 2) { value = static_cast<float>(static_cast<int16_t>(be16(data))); return true; }
      break;
    case DLMS_TYPE_UINT32:
      if (avail >= 4) { value = static_cast<float>(be32(data)); return true; }
      break;
    case DLMS_TYPE_INT32:
      if (avail >= 4) { value = static_cast<float>(static_cast<int32_t>(be32(data))); return true; }
      break;
    case DLMS_TYPE_UINT64: {
      if (avail >= 8) {
        uint64_t v = 0;
        for (int i = 0; i < 8; i++) v = (v << 8) | data[i];
        value = static_cast<float>(v);
        return true;
      }
      break;
    }
    case DLMS_TYPE_INT64: {
      if (avail >= 8) {
        uint64_t v = 0;
        for (int i = 0; i < 8; i++) v = (v << 8) | data[i];
        value = static_cast<float>(static_cast<int64_t>(v));
        return true;
      }
      break;
    }
    case DLMS_TYPE_FLOAT32:
      if (avail >= 4) {
        uint32_t v = be32(data);
        float f;
        memcpy(&f, &v, sizeof(float));
        value = f;
        return true;
      }
      break;
    case DLMS_TYPE_FLOAT64:
      if (avail >= 8) {
        uint64_t v = 0;
        for (int i = 0; i < 8; i++) v = (v << 8) | data[i];
        double d;
        memcpy(&d, &v, sizeof(double));
        value = static_cast<float>(d);
        return true;
      }
      break;
    case DLMS_TYPE_ENUM:
    case DLMS_TYPE_BOOLEAN:
      if (avail >= 1) { value = static_cast<float>(data[0]); return true; }
      break;

    // --- String/octet types (length-prefixed) ---
    case DLMS_TYPE_OCTET_STRING: {
      if (avail < 1) break;
      uint8_t slen = data[0];
      if (avail < 1u + slen) break;
      const uint8_t *p = &data[1];
      is_text = true;
      if (slen == 12) {
        // DLMS datetime: YYYY-MM-DD HH:MM:SS
        char buf[24];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                 be16(p), p[2], p[3], p[5], p[6], p[7]);
        text_value = buf;
      } else if (slen == 5) {
        // DLMS date: YYYY-MM-DD
        char buf[12];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d", be16(p), p[2], p[3]);
        text_value = buf;
      } else if (slen == 4) {
        // DLMS time: HH:MM:SS
        char buf[12];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", p[0], p[1], p[2]);
        text_value = buf;
      } else {
        // Generic octet-string → hex
        char hex[3];
        text_value.clear();
        text_value.reserve(slen * 2);
        for (uint8_t i = 0; i < slen; i++) {
          snprintf(hex, sizeof(hex), "%02X", p[i]);
          text_value += hex;
        }
      }
      return true;
    }
    case DLMS_TYPE_VISIBLE_STRING:
    case DLMS_TYPE_UTF8_STRING: {
      if (avail < 1) break;
      uint8_t slen = data[0];
      if (avail < 1u + slen) break;
      is_text = true;
      text_value = std::string(reinterpret_cast<const char *>(&data[1]), slen);
      return true;
    }

    default:
      break;
  }

  return false;
}

bool NartisWmbusComponent::parse_get_response_(const uint8_t *data, uint16_t len,
                                                float &value, std::string &text_value,
                                                bool &is_text) {
  is_text = false;

  if (len < 3 || data[0] != DLMS_TAG_GET_RESPONSE) {
    ESP_LOGW(TAG, "Not a GET.response (tag=0x%02X)", data[0]);
    return false;
  }

  // data[1] = response type (0x01 = normal), data[2] = invoke-id
  uint16_t pos = 3;
  if (pos >= len) return false;

  // Data-Access-Result or Data
  uint8_t choice = data[pos++];
  if (choice == 0x01) {
    if (pos < len)
      ESP_LOGW(TAG, "GET.response error: access-result=%d", data[pos]);
    return false;
  }
  if (choice != 0x00) {
    ESP_LOGW(TAG, "GET.response unexpected choice=%d", choice);
    return false;
  }

  if (pos >= len) return false;
  uint8_t type_tag = data[pos++];

  if (!dlms_decode_data_element(type_tag, &data[pos], len - pos, value, text_value, is_text)) {
    ESP_LOGD(TAG, "GET.response: failed to decode DLMS type 0x%02X", type_tag);
    return false;
  }
  return true;
}

// ============================================================================
// High-level TX/RX
// ============================================================================

bool NartisWmbusComponent::transmit_dlms_(const uint8_t *apdu, uint16_t apdu_len,
                                            uint8_t c_field, bool encrypt) {
  uint8_t frame_buf[MAX_FRAME_SIZE];

  // Increment access number for each TX (per firmware frame_set_mode 0xBAF4)
  access_nr_++;

  if (encrypt && system_title_valid_) {
    // Encrypted: wmbus_encrypt_ produces SC + pad + IC + ciphertext
    uint8_t enc_payload[MAX_APDU_SIZE];
    uint16_t enc_len = wmbus_encrypt_(apdu, apdu_len, enc_payload);
    if (enc_len == 0)
      return false;
    uint16_t frame_len = wmbus_frame_build_(c_field, WMBUS_CI_ENC, enc_payload, enc_len, frame_buf);
    return radio_.send_packet(frame_buf, frame_len, channel_);
  } else {
    // Unencrypted: plain APDU goes directly after TPL header
    uint16_t frame_len = wmbus_frame_build_(c_field, WMBUS_CI_PLAIN, apdu, apdu_len, frame_buf);
    return radio_.send_packet(frame_buf, frame_len, channel_);
  }
}

// Process raw RF packet into DLMS APDU (pure data, no radio calls)
uint16_t NartisWmbusComponent::process_rx_frame_(const uint8_t *rf_buf, uint16_t rf_len,
                                                   uint8_t *dlms_out) {
  // Strip W-MBus CRC blocks
  uint8_t stripped[MAX_APDU_SIZE + 30];
  uint16_t stripped_len = wmbus_frame_parse_(rf_buf, rf_len, stripped);
  if (stripped_len == 0) {
    ESP_LOGW(TAG, "Failed to parse W-MBus frame");
    return 0;
  }

  // Parse W-MBus DLL header: L(1) + C(1) + M(2) + A(6) + CI(1) = 11 bytes
  if (stripped_len < 11) {
    ESP_LOGW(TAG, "Frame too short: %d", stripped_len);
    return 0;
  }

  uint8_t ci_field = stripped[10];

  ESP_LOGD(TAG, "RX frame: L=%d C=0x%02X CI=0x%02X total=%d",
           stripped[0], stripped[1], ci_field, stripped_len);

  // After CI, firmware expects TPL short header (4 bytes):
  //   C_copy(1) + CI_copy(1) + access_nr(1) + status(1)
  // Then encrypted block or plaintext DLMS APDU
  // Per firmware frame_parse (0xBF5E): data after DLL+TPL header
  static constexpr uint16_t DLL_HDR_LEN = 11;  // L+C+M+A+CI
  static constexpr uint16_t TPL_SHORT_HDR_LEN = 4;  // C_copy+CI_copy+access_nr+status
  static constexpr uint16_t FULL_HDR_LEN = DLL_HDR_LEN + TPL_SHORT_HDR_LEN;  // 15

  if (ci_field == 0x7A) {
    // CI=0x7A: TPL short header present
    if (stripped_len < FULL_HDR_LEN) {
      ESP_LOGW(TAG, "Frame too short for TPL header: %d", stripped_len);
      return 0;
    }

    uint8_t *data_after_tpl = &stripped[FULL_HDR_LEN];
    uint16_t data_len = stripped_len - FULL_HDR_LEN;

    ESP_LOGD(TAG, "  TPL: C_copy=0x%02X CI_copy=0x%02X acc=%d status=0x%02X",
             stripped[11], stripped[12], stripped[13], stripped[14]);

    if (data_len == 0)
      return 0;

    // Check if encrypted: SC byte == 0x94 (AES-GCM)
    if (data_after_tpl[0] == WMBUS_SC_GCM) {
      return wmbus_decrypt_(data_after_tpl, data_len, dlms_out);
    } else {
      // Plaintext DLMS APDU (e.g. AARE response)
      memcpy(dlms_out, data_after_tpl, data_len);
      return data_len;
    }
  }

  // Fallback: handle legacy CI values without TPL header
  uint8_t *payload = &stripped[DLL_HDR_LEN];
  uint16_t payload_len = stripped_len - DLL_HDR_LEN;

  if (ci_field == WMBUS_CI_ENC) {
    return wmbus_decrypt_(payload, payload_len, dlms_out);
  } else if (ci_field == WMBUS_CI_PLAIN) {
    memcpy(dlms_out, payload, payload_len);
    return payload_len;
  }

  ESP_LOGW(TAG, "Unknown CI field: 0x%02X", ci_field);
  return 0;
}

// Blocking receive — only used by sniffer mode
uint16_t NartisWmbusComponent::receive_dlms_(uint8_t *dlms_out, uint32_t timeout_ms) {
  uint8_t rf_buf[MAX_FRAME_SIZE];
  uint16_t rf_len = radio_.receive_packet(rf_buf, sizeof(rf_buf), timeout_ms, channel_);
  if (rf_len == 0)
    return 0;
  return process_rx_frame_(rf_buf, rf_len, dlms_out);
}

// Invocation counter starts at 0 each boot — no NVS persistence needed.
// The Nartis meter does not enforce monotonic IC (original CIU EEPROM has no
// persistent IC storage either — placeholder system title confirms this).

// ============================================================================
// Sensor Registration
// ============================================================================

void NartisWmbusComponent::register_sensor(NartisWmbusSensorBase *sensor) {
  this->sensors_.insert({sensor->get_obis_code(), sensor});
}

// ============================================================================
// Component Lifecycle
// ============================================================================

void NartisWmbusComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Nartis W-MBus component...");

  // Initialize radio
  radio_.set_pins(pin_sdio_, pin_sclk_, pin_csb_, pin_fcsb_, pin_gpio1_);

  this->set_timeout(2000, [this]() {
    if (!this->radio_.init(this->channel_)) {
      ESP_LOGE(TAG, "Radio init failed");
      this->mark_failed();
      return;
    }

    float freq = CMT2300A_FREQ_MHZ[this->channel_ < 4 ? this->channel_ : 1];
    switch (this->mode_) {
      case Mode::SNIFFER:
        ESP_LOGI(TAG, "Entering SNIFFER mode on ch %d (%.3f MHz)", this->channel_, freq);
        this->state_ = State::SNIFFING;
        break;
      case Mode::LISTEN:
        // In listen mode, system_title config = meter's system title (for decryption)
        memcpy(this->meter_system_title_, this->system_title_.data(), 8);
        this->system_title_valid_ = true;
        ESP_LOGI(TAG, "Entering LISTEN mode on ch %d (%.3f MHz), sys_title=%02X%02X%02X%02X%02X%02X%02X%02X",
                 this->channel_, freq,
                 this->meter_system_title_[0], this->meter_system_title_[1],
                 this->meter_system_title_[2], this->meter_system_title_[3],
                 this->meter_system_title_[4], this->meter_system_title_[5],
                 this->meter_system_title_[6], this->meter_system_title_[7]);
        this->state_ = State::LISTENING;
        break;
      default:
        ESP_LOGD(TAG, "Radio ready, waiting for first poll");
        this->state_ = State::IDLE;
        break;
    }
  });
}

void NartisWmbusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Nartis W-MBus:");
  ESP_LOGCONFIG(TAG, "  Channel: %d (%.3f MHz)", channel_,
                CMT2300A_FREQ_MHZ[channel_ < 4 ? channel_ : 1]);
  ESP_LOGCONFIG(TAG, "  AES Key: %02X%02X%02X%02X...%02X%02X%02X%02X",
                decryption_key_[0], decryption_key_[1], decryption_key_[2], decryption_key_[3],
                decryption_key_[12], decryption_key_[13], decryption_key_[14], decryption_key_[15]);
  ESP_LOGCONFIG(TAG, "  DLMS Password: %s", DLMS_PASSWORD);
  ESP_LOGCONFIG(TAG, "  Client SAP: 0x%02X", DLMS_CLIENT_SAP);
  ESP_LOGCONFIG(TAG, "  Invocation Counter: %u", invocation_counter_);
  ESP_LOGCONFIG(TAG, "  Mode: %s", LOG_STR_ARG(mode_to_string_(mode_)));
  ESP_LOGCONFIG(TAG, "  Sensors: %d", sensors_.size());
  LOG_UPDATE_INTERVAL(this);
}

void NartisWmbusComponent::update() {
  if (mode_ == Mode::SNIFFER) {
    ESP_LOGD(TAG, "Sniffer: %u packets captured so far", sniffer_packet_count_);
    return;
  }

  if (mode_ == Mode::LISTEN) {
    ESP_LOGD(TAG, "Listen: %u packets received", listen_packet_count_);
    // TODO: publish sensor values once push data parsing is implemented
    return;
  }

  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Update skipped — not idle (state=%s)", LOG_STR_ARG(state_to_string_(state_)));
    return;
  }

  if (sensors_.empty()) {
    ESP_LOGW(TAG, "No sensors configured");
    return;
  }

  ESP_LOGD(TAG, "Starting data collection session");
  set_next_state_(State::INIT_RADIO);
}

void NartisWmbusComponent::set_next_state_(State next_state) {
  if (state_ != next_state) {
    ESP_LOGD(TAG, "State: %s -> %s", LOG_STR_ARG(state_to_string_(state_)), LOG_STR_ARG(state_to_string_(next_state)));
  }
  state_ = next_state;
}

const LogString *NartisWmbusComponent::state_to_string_(State state) {
  switch (state) {
    case State::NOT_INITIALIZED: return LOG_STR("NOT_INITIALIZED");
    case State::IDLE:            return LOG_STR("IDLE");
    case State::INIT_RADIO:      return LOG_STR("INIT_RADIO");
    case State::SEND_AARQ:       return LOG_STR("SEND_AARQ");
    case State::WAIT_AARE:       return LOG_STR("WAIT_AARE");
    case State::DATA_REQUEST:    return LOG_STR("DATA_REQUEST");
    case State::WAIT_RESPONSE:   return LOG_STR("WAIT_RESPONSE");
    case State::DATA_NEXT:       return LOG_STR("DATA_NEXT");
    case State::SEND_RELEASE:    return LOG_STR("SEND_RELEASE");
    case State::WAIT_RELEASE:    return LOG_STR("WAIT_RELEASE");
    case State::PUBLISH:         return LOG_STR("PUBLISH");
    case State::LISTENING:       return LOG_STR("LISTENING");
    case State::SNIFFING:        return LOG_STR("SNIFFING");
    default:                     return LOG_STR("UNKNOWN");
  }
}

// ============================================================================
// Main FSM Loop
// ============================================================================

void NartisWmbusComponent::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED)
    return;

  // Session watchdog — force back to IDLE if stuck
  if (this->state_ != State::IDLE && this->state_ != State::SNIFFING &&
      this->state_ != State::LISTENING && this->state_ != State::INIT_RADIO &&
      check_session_timeout_()) {
    ESP_LOGW(TAG, "Session timeout (%ums) in state %s — aborting",
             SESSION_TIMEOUT_MS, LOG_STR_ARG(state_to_string_(state_)));
    radio_.go_standby();

    set_next_state_(State::IDLE);
    return;
  }

  switch (this->state_) {
    case State::IDLE:
      break;

    case State::INIT_RADIO: {
      session_start_ms_ = millis();

      // Reset sensor states
      for (auto &pair : sensors_) {
        pair.second->reset();
      }

      system_title_valid_ = false;
      retry_count_ = 0;
      set_next_state_(State::SEND_AARQ);
      break;
    }

    case State::SEND_AARQ: {
      ESP_LOGD(TAG, "Sending AARQ (unencrypted, SND-IR)");

      // AARQ is sent unencrypted via SND-IR to establish association
      if (!transmit_dlms_(AARQ_TEMPLATE, sizeof(AARQ_TEMPLATE), WMBUS_C_SND_IR, false)) {
        ESP_LOGW(TAG, "Failed to send AARQ");
        set_next_state_(State::IDLE);
        break;
      }

      retry_count_ = 0;
      if (!radio_.start_rx(channel_)) {
        ESP_LOGW(TAG, "Failed to start RX for AARE");
        set_next_state_(State::IDLE);
        break;
      }
      start_timeout_(AARE_TIMEOUT_MS);
      set_next_state_(State::WAIT_AARE);
      break;
    }

    case State::WAIT_AARE: {
      // Non-blocking: poll radio each loop() iteration
      if (check_timeout_()) {
        radio_.stop_rx();
        retry_count_++;
        if (retry_count_ >= MAX_RETRIES) {
          ESP_LOGW(TAG, "No AARE after %d retries", MAX_RETRIES);
          set_next_state_(State::IDLE);
        } else {
          ESP_LOGD(TAG, "AARE timeout, retry %d/%d", retry_count_, MAX_RETRIES);
          set_next_state_(State::SEND_AARQ);
        }
        break;
      }

      uint8_t rf_buf[MAX_FRAME_SIZE];
      int16_t rf_len = radio_.check_rx(rf_buf, sizeof(rf_buf));
      if (rf_len == 0)
        break;  // nothing yet — return to loop()
      radio_.stop_rx();

      if (rf_len < 0) {
        // RX error — restart RX and wait more
        radio_.start_rx(channel_);
        break;
      }

      uint16_t len = process_rx_frame_(rf_buf, rf_len, apdu_buf_);
      if (len == 0) {
        // Bad frame — restart RX and keep waiting
        radio_.start_rx(channel_);
        break;
      }

      if (!parse_aare_(apdu_buf_, len)) {
        ESP_LOGW(TAG, "AARE parse failed");
        set_next_state_(State::IDLE);
        break;
      }

      // Start data collection from first sensor
      request_iter_ = sensors_.begin();
      set_next_state_(State::DATA_REQUEST);
      break;
    }

    case State::DATA_REQUEST: {
      if (request_iter_ == sensors_.end()) {
        set_next_state_(State::SEND_RELEASE);
        break;
      }

      NartisWmbusSensorBase *sensor = request_iter_->second;
      current_obis_ = sensor->get_obis_code();

      uint8_t obis_bytes[6];
      sensor->parse_obis_bytes(obis_bytes);

      uint8_t get_req[20];
      uint16_t req_len = build_get_request_(obis_bytes, sensor->get_class_id(),
                                             sensor->get_attribute(), get_req);

      ESP_LOGD(TAG, "GET.request for OBIS %s (class=%d, attr=%d)",
               current_obis_.c_str(), sensor->get_class_id(), sensor->get_attribute());

      if (!transmit_dlms_(get_req, req_len, WMBUS_C_SND_NR, true)) {
        ESP_LOGW(TAG, "Failed to send GET.request");
        sensor->record_failure();
        set_next_state_(State::DATA_NEXT);
        break;
      }

      retry_count_ = 0;
      if (!radio_.start_rx(channel_)) {
        ESP_LOGW(TAG, "Failed to start RX for response");
        sensor->record_failure();
        set_next_state_(State::DATA_NEXT);
        break;
      }
      start_timeout_(RESPONSE_TIMEOUT_MS);
      set_next_state_(State::WAIT_RESPONSE);
      break;
    }

    case State::WAIT_RESPONSE: {
      // Non-blocking: poll radio each loop() iteration
      if (check_timeout_()) {
        radio_.stop_rx();
        retry_count_++;
        if (retry_count_ >= MAX_RETRIES) {
          ESP_LOGW(TAG, "No response for OBIS %s after %d retries", current_obis_.c_str(), MAX_RETRIES);
          auto range = sensors_.equal_range(current_obis_);
          for (auto it = range.first; it != range.second; ++it) {
            it->second->record_failure();
          }
          set_next_state_(State::DATA_NEXT);
        } else {
          ESP_LOGD(TAG, "Response timeout, retry %d/%d", retry_count_, MAX_RETRIES);
          set_next_state_(State::DATA_REQUEST);
        }
        break;
      }

      uint8_t rf_buf[MAX_FRAME_SIZE];
      int16_t rf_len = radio_.check_rx(rf_buf, sizeof(rf_buf));
      if (rf_len == 0)
        break;  // nothing yet — return to loop()
      radio_.stop_rx();

      if (rf_len < 0) {
        // RX error — restart RX and wait more
        radio_.start_rx(channel_);
        break;
      }

      uint16_t len = process_rx_frame_(rf_buf, rf_len, apdu_buf_);
      if (len == 0) {
        // Bad frame — restart RX and keep waiting
        radio_.start_rx(channel_);
        break;
      }

      // Parse response and distribute to all sensors with this OBIS code
      float value = 0.0f;
      std::string text_value;
      bool is_text = false;
      if (parse_get_response_(apdu_buf_, len, value, text_value, is_text)) {
        if (is_text) {
          ESP_LOGD(TAG, "OBIS %s = \"%s\"", current_obis_.c_str(), text_value.c_str());
        } else {
          ESP_LOGD(TAG, "OBIS %s = %.3f", current_obis_.c_str(), value);
        }
        auto range = sensors_.equal_range(current_obis_);
        for (auto it = range.first; it != range.second; ++it) {
          if (it->second->get_type() == SENSOR_NUMERIC && !is_text) {
            static_cast<NartisWmbusSensor *>(it->second)->set_value(value);
#ifdef USE_TEXT_SENSOR
          } else if (it->second->get_type() == SENSOR_TEXT && is_text) {
            static_cast<NartisWmbusTextSensor *>(it->second)->set_value(text_value.c_str());
          } else if (it->second->get_type() == SENSOR_TEXT && !is_text) {
            // Numeric value to text sensor — convert to string
            char buf[16];
            snprintf(buf, sizeof(buf), "%.3f", value);
            static_cast<NartisWmbusTextSensor *>(it->second)->set_value(buf);
#endif
          }
        }
      } else {
        ESP_LOGW(TAG, "Failed to parse GET.response for OBIS %s", current_obis_.c_str());
        auto range = sensors_.equal_range(current_obis_);
        for (auto it = range.first; it != range.second; ++it) {
          it->second->record_failure();
        }
      }

      set_next_state_(State::DATA_NEXT);
      break;
    }

    case State::DATA_NEXT: {
      // Advance to next unique OBIS code
      std::string last_obis = current_obis_;
      while (request_iter_ != sensors_.end() && request_iter_->first == last_obis) {
        ++request_iter_;
      }

      if (request_iter_ == sensors_.end()) {
        set_next_state_(State::SEND_RELEASE);
      } else {
        set_next_state_(State::DATA_REQUEST);
      }
      break;
    }

    case State::SEND_RELEASE: {
      ESP_LOGD(TAG, "Sending RLRQ");
      transmit_dlms_(RLRQ_TEMPLATE, sizeof(RLRQ_TEMPLATE), WMBUS_C_SND_NR, true);

      if (!radio_.start_rx(channel_)) {
        // Can't RX — just finish up
    
        set_next_state_(State::PUBLISH);
        break;
      }
      start_timeout_(RELEASE_TIMEOUT_MS);
      set_next_state_(State::WAIT_RELEASE);
      break;
    }

    case State::WAIT_RELEASE: {
      // Non-blocking: poll for RLRE, but don't care if it times out
      if (check_timeout_()) {
        radio_.stop_rx();
    
        set_next_state_(State::PUBLISH);
        break;
      }

      uint8_t rf_buf[MAX_FRAME_SIZE];
      int16_t rf_len = radio_.check_rx(rf_buf, sizeof(rf_buf));
      if (rf_len == 0)
        break;  // nothing yet — return to loop()

      // Got something (RLRE or whatever) — done
      radio_.stop_rx();
  
      set_next_state_(State::PUBLISH);
      break;
    }

    case State::PUBLISH: {
      ESP_LOGD(TAG, "Publishing sensor values");
      for (auto &pair : sensors_) {
        if (pair.second->has_value()) {
          pair.second->publish();
        }
      }
      set_next_state_(State::IDLE);
      break;
    }

    case State::LISTENING: {
      listen_loop_();
      break;
    }

    case State::SNIFFING: {
      sniff_loop_();
      break;
    }

    default:
      set_next_state_(State::IDLE);
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
    case 0x44: return LOG_STR("SND-NR");
    case 0x46: return LOG_STR("SND-IR");
    case 0x08: return LOG_STR("RSP-UD");
    case 0x00: return LOG_STR("SND-NKE");
    case 0x40: return LOG_STR("SND-UD");
    default:   return LOG_STR("???");
  }
}

const LogString *NartisWmbusComponent::ci_field_to_string_(uint8_t ci_field) {
  switch (ci_field) {
    case 0xBD: return LOG_STR("ENC-DLMS");
    case 0xC5: return LOG_STR("PLAIN");
    case 0x72: return LOG_STR("RSP-UD/12B");
    case 0x78: return LOG_STR("RSP-UD/0B");
    default:   return LOG_STR("???");
  }
}

void NartisWmbusComponent::decode_manufacturer_(uint16_t m_field, char out[4]) {
  out[0] = static_cast<char>(((m_field >> 10) & 0x1F) + 64);
  out[1] = static_cast<char>(((m_field >> 5) & 0x1F) + 64);
  out[2] = static_cast<char>((m_field & 0x1F) + 64);
  out[3] = '\0';
}

void NartisWmbusComponent::log_raw_frame_(const uint8_t *data, uint16_t len) {
  // Log raw hex in chunks (ESP_LOG has ~256 char limit)
  char hex_buf[129];  // 64 bytes per line
  uint16_t pos = 0;
  ESP_LOGD(TAG, "SNIFF raw [%d bytes]:", len);
  while (pos < len) {
    uint16_t chunk = len - pos;
    if (chunk > 64) chunk = 64;
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
  decode_manufacturer_(m_field, mfr);

  ESP_LOGI(TAG, "SNIFF #%u: L=%d C=0x%02X(%s) M=%s(0x%04X) A=%08X v=%d t=0x%02X CI=0x%02X(%s)",
           sniffer_packet_count_, l_field, c_field, LOG_STR_ARG(c_field_to_string_(c_field)),
           mfr, m_field, serial, version, dev_type,
           ci_field, LOG_STR_ARG(ci_field_to_string_(ci_field)));

  // Parse payload depending on CI
  // CI=0x7A: TPL short header (4 bytes) + data
  // Other CI: data directly after CI
  uint16_t data_offset = 11;  // after DLL header (L+C+M+A+CI)
  uint16_t tpl_hdr_len = 0;

  if (ci_field == 0x7A && len >= 15) {
    // TPL short header: C_copy + CI_copy + access_nr + status
    tpl_hdr_len = 4;
    data_offset = 15;
    ESP_LOGI(TAG, "  TPL: C=0x%02X CI=0x%02X acc=%d status=0x%02X",
             frame[11], frame[12], frame[13], frame[14]);
  }

  if (data_offset >= len)
    return;

  const uint8_t *data = &frame[data_offset];
  uint16_t data_len = len - data_offset;

  // Check if encrypted: first byte is SC=0x94
  if (data_len >= 7 && data[0] == WMBUS_SC_GCM) {
    // Encrypted: SC(1) + pad(1) + IC(4) + ciphertext
    uint8_t sc = data[0];
    uint32_t ic = ((uint32_t) data[2] << 24) | ((uint32_t) data[3] << 16) |
                  ((uint32_t) data[4] << 8) | data[5];
    uint16_t ct_len = data_len - 6;

    ESP_LOGI(TAG, "  ENC: SC=0x%02X IC=0x%08X(%u) ciphertext=%d bytes",
             sc, ic, ic, ct_len);

    // Try decryption if we have the key and a system title
    if (ct_len > 0 && ct_len < MAX_APDU_SIZE) {
      uint8_t nonce[12];
      bool can_decrypt = false;
      if (system_title_valid_) {
        memcpy(nonce, meter_system_title_, 8);
        can_decrypt = true;
      }
      if (can_decrypt) {
        nonce[8] = (ic >> 24) & 0xFF;
        nonce[9] = (ic >> 16) & 0xFF;
        nonce[10] = (ic >> 8) & 0xFF;
        nonce[11] = ic & 0xFF;

        uint8_t plain[MAX_APDU_SIZE];
        uint8_t sc_aad = data[0];  // SC byte as AAD
        if (aes_gcm_crypt_(false, decryption_key_.data(), nonce, &sc_aad, 1, &data[6], ct_len, plain)) {
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

    // Check if this is an AARE (tag 0x61) — extract system title for future decryption
    if (data_len > 2 && data[0] == 0x61) {
      ESP_LOGI(TAG, "  Detected AARE — attempting system title extraction");
      if (parse_aare_(data, data_len)) {
        ESP_LOGI(TAG, "  System title captured: %02X%02X%02X%02X%02X%02X%02X%02X",
                 meter_system_title_[0], meter_system_title_[1],
                 meter_system_title_[2], meter_system_title_[3],
                 meter_system_title_[4], meter_system_title_[5],
                 meter_system_title_[6], meter_system_title_[7]);
      }
    }
  }
}

void NartisWmbusComponent::sniff_loop_() {
  // Non-blocking RX poll — short timeout so loop() stays responsive
  uint8_t rf_buf[MAX_FRAME_SIZE];
  uint16_t rf_len = radio_.receive_packet(rf_buf, sizeof(rf_buf), 100, channel_);
  if (rf_len == 0)
    return;

  sniffer_packet_count_++;

  // Log raw frame
  log_raw_frame_(rf_buf, rf_len);

  // Try to strip CRCs and parse
  uint8_t stripped[MAX_APDU_SIZE + 20];
  uint16_t stripped_len = wmbus_frame_parse_(rf_buf, rf_len, stripped);

  if (stripped_len > 0) {
    ESP_LOGI(TAG, "SNIFF CRC: OK (raw=%d, clean=%d)", rf_len, stripped_len);
    log_parsed_frame_(stripped, stripped_len);
  } else {
    ESP_LOGW(TAG, "SNIFF CRC: FAILED (raw=%d bytes) — not a valid W-MBus frame", rf_len);
  }
}

// ============================================================================
// Listen Mode
// ============================================================================

const LogString *NartisWmbusComponent::mode_to_string_(Mode mode) {
  switch (mode) {
    case Mode::SESSION: return LOG_STR("SESSION");
    case Mode::LISTEN:  return LOG_STR("LISTEN");
    case Mode::SNIFFER: return LOG_STR("SNIFFER");
    default:            return LOG_STR("UNKNOWN");
  }
}

void NartisWmbusComponent::listen_loop_() {
  uint8_t rf_buf[MAX_FRAME_SIZE];
  uint16_t rf_len = radio_.receive_packet(rf_buf, sizeof(rf_buf), 100, channel_);
  if (rf_len == 0)
    return;

  listen_packet_count_++;

  // Strip CRCs and log header
  uint8_t stripped[MAX_FRAME_SIZE];
  uint16_t stripped_len = wmbus_frame_parse_(rf_buf, rf_len, stripped);
  if (stripped_len < 11) {
    ESP_LOGD(TAG, "LISTEN #%u: bad frame (%d raw bytes)", listen_packet_count_, rf_len);
    return;
  }

  // Log W-MBus header
  uint16_t m_field = stripped[2] | (stripped[3] << 8);
  uint32_t serial = stripped[4] | (stripped[5] << 8) | (stripped[6] << 16) | (stripped[7] << 24);
  char mfr[4];
  decode_manufacturer_(m_field, mfr);
  ESP_LOGI(TAG, "LISTEN #%u: C=0x%02X(%s) M=%s S=%08X v=%d t=0x%02X CI=0x%02X(%s)",
           listen_packet_count_, stripped[1], LOG_STR_ARG(c_field_to_string_(stripped[1])),
           mfr, serial, stripped[8], stripped[9],
           stripped[10], LOG_STR_ARG(ci_field_to_string_(stripped[10])));

  // Extract DLMS APDU (re-parses CRCs internally — minor overhead)
  uint16_t dlms_len = process_rx_frame_(rf_buf, rf_len, apdu_buf_);
  if (dlms_len == 0) {
    ESP_LOGD(TAG, "LISTEN: no DLMS payload");
    return;
  }

  // Log DLMS APDU hex
  char hex[129];
  uint16_t dump = dlms_len > 64 ? 64 : dlms_len;
  hex_to_str_(apdu_buf_, dump, hex, sizeof(hex));
  ESP_LOGI(TAG, "LISTEN: DLMS [%d]: %s%s", dlms_len, hex, dlms_len > 64 ? "..." : "");

  // Parse and extract values
  listen_parse_dlms_(apdu_buf_, dlms_len);
}

void NartisWmbusComponent::listen_parse_dlms_(const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;

  // Log known APDU tags for context
  uint8_t tag = data[0];
  switch (tag) {
    case 0x0C: ESP_LOGI(TAG, "LISTEN: data-notification"); break;
    case 0xC4: ESP_LOGI(TAG, "LISTEN: GET.response"); break;
    case 0xC5: ESP_LOGI(TAG, "LISTEN: SET.response"); break;
    case 0xC7: ESP_LOGI(TAG, "LISTEN: ACTION.response"); break;
    case 0x61: {
      ESP_LOGI(TAG, "LISTEN: AARE — extracting system title");
      parse_aare_(data, len);
      return;
    }
    default: ESP_LOGI(TAG, "LISTEN: DLMS tag 0x%02X", tag); break;
  }

  // TODO: parse push data structure once format is known
  // For now, full hex dump is logged by listen_loop_
}

void NartisWmbusComponent::scan_obis_values_(const uint8_t * /*data*/, uint16_t /*len*/) {
  // TODO: implement OBIS+value extraction from push data once format is known
}

}  // namespace nartis_wmbus
}  // namespace esphome
