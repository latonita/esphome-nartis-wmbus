#include "dlms_helpers.h"
#include "wmbus_protocol.h"

#include "esphome/core/log.h"

#include "mbedtls/esp_config.h"
#include "mbedtls/gcm.h"

#include "miniz.h"

#include <cstdio>
#include <cstring>

namespace esphome::nartis_wmbus {

const uint8_t AARQ_TEMPLATE[] = {
    0x60, 0x34, 0xA1, 0x09, 0x06, 0x07, 0x60, 0x85, 0x74, 0x05, 0x08, 0x01, 0x01, 0x8A, 0x02, 0x07, 0x80, 0x8B,
    0x07, 0x60, 0x85, 0x74, 0x05, 0x08, 0x02, 0x01, 0xAC, 0x08, 0x80, 0x06, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
    0xBE, 0x10, 0x04, 0x0E, 0x01, 0x00, 0x00, 0x00, 0x07, 0x5F, 0x1F, 0x04, 0x00, 0x00, 0x7E, 0x1F, 0x01, 0x2C,
};
const uint16_t AARQ_TEMPLATE_SIZE = sizeof(AARQ_TEMPLATE);

uint16_t dlms_build_get_request(const uint8_t obis[6], uint16_t class_id, uint8_t attr, uint8_t *out) {
  uint16_t pos = 0;
  out[pos++] = 0xC0;
  out[pos++] = 0x01;
  out[pos++] = 0xC0;
  out[pos++] = (class_id >> 8) & 0xFF;
  out[pos++] = class_id & 0xFF;
  memcpy(&out[pos], obis, 6);
  pos += 6;
  out[pos++] = attr;
  out[pos++] = 0x00;
  return pos;
}

bool dlms_parse_aare(const char *log_tag, const uint8_t *data, uint16_t len, uint8_t meter_system_title[8],
                     bool &system_title_valid) {
  if (len < 2 || data[0] != DLMS_TAG_AARE) {
    ESP_LOGW(log_tag, "Not an AARE (tag=0x%02X)", data[0]);
    return false;
  }

  bool result_ok = false;
  bool found_system_title = false;
  uint16_t pos = 2;

  while (pos < len) {
    uint8_t tag = data[pos];
    if (pos + 1 >= len)
      break;
    uint8_t tag_len = data[pos + 1];
    pos += 2;
    if (pos + tag_len > len)
      break;

    if (tag == DLMS_TAG_AARE_RESULT) {
      if (tag_len >= DLMS_AARE_RESULT_FIELD_MIN_LEN && data[pos] == ASN1_TAG_INTEGER &&
          data[pos + 1] == ASN1_INTEGER_U8_LEN) {
        uint8_t result = data[pos + 2];
        if (result == DLMS_ASSOCIATION_RESULT_ACCEPTED) {
          result_ok = true;
          ESP_LOGD(log_tag, "AARE: association accepted");
        } else {
          ESP_LOGW(log_tag, "AARE: association rejected (result=%d)", result);
          return false;
        }
      }
    } else if (tag == DLMS_TAG_AARE_RESPONDING_AP_TITLE) {
      if (tag_len >= DLMS_AARE_AP_TITLE_FIELD_LEN && data[pos] == ASN1_TAG_OCTET_STRING &&
          data[pos + 1] == DLMS_SYSTEM_TITLE_SIZE) {
        memcpy(meter_system_title, &data[pos + 2], DLMS_SYSTEM_TITLE_SIZE);
        system_title_valid = true;
        found_system_title = true;
        ESP_LOGI(log_tag, "************************************************************");
        ESP_LOGI(log_tag, "  Meter system title: %02X%02X%02X%02X%02X%02X%02X%02X", meter_system_title[0],
                 meter_system_title[1], meter_system_title[2], meter_system_title[3], meter_system_title[4],
                 meter_system_title[5], meter_system_title[6], meter_system_title[7]);
        ESP_LOGI(log_tag, "  Add to YAML:  meter_system_title: \"%02X%02X%02X%02X%02X%02X%02X%02X\"",
                 meter_system_title[0], meter_system_title[1], meter_system_title[2], meter_system_title[3],
                 meter_system_title[4], meter_system_title[5], meter_system_title[6], meter_system_title[7]);
        ESP_LOGI(log_tag, "************************************************************");
      }
    }

    pos += tag_len;
  }

  if (!result_ok) {
    ESP_LOGW(log_tag, "AARE: no result field found");
    return false;
  }
  if (!found_system_title) {
    ESP_LOGW(log_tag, "AARE: no system title found (will try without)");
  }

  return true;
}

static bool dlms_decode_data_element(uint8_t type_tag, const uint8_t *data, uint16_t avail, float &value,
                                     char *text_buf, uint16_t text_buf_size, bool &is_text) {
  is_text = false;

  auto be16 = [](const uint8_t *p) -> uint16_t { return static_cast<uint16_t>((p[0] << 8) | p[1]); };
  auto be32 = [](const uint8_t *p) -> uint32_t {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
  };

  switch (type_tag) {
    case DLMS_TYPE_UINT8:
      if (avail >= 1) {
        value = static_cast<float>(data[0]);
        return true;
      }
      break;
    case DLMS_TYPE_INT8:
      if (avail >= 1) {
        value = static_cast<float>(static_cast<int8_t>(data[0]));
        return true;
      }
      break;
    case DLMS_TYPE_UINT16:
      if (avail >= 2) {
        value = static_cast<float>(be16(data));
        return true;
      }
      break;
    case DLMS_TYPE_INT16:
      if (avail >= 2) {
        value = static_cast<float>(static_cast<int16_t>(be16(data)));
        return true;
      }
      break;
    case DLMS_TYPE_UINT32:
      if (avail >= 4) {
        value = static_cast<float>(be32(data));
        return true;
      }
      break;
    case DLMS_TYPE_INT32:
      if (avail >= 4) {
        value = static_cast<float>(static_cast<int32_t>(be32(data)));
        return true;
      }
      break;
    case DLMS_TYPE_UINT64: {
      if (avail >= 8) {
        uint64_t v = 0;
        for (int i = 0; i < 8; i++)
          v = (v << 8) | data[i];
        value = static_cast<float>(v);
        return true;
      }
      break;
    }
    case DLMS_TYPE_INT64: {
      if (avail >= 8) {
        uint64_t v = 0;
        for (int i = 0; i < 8; i++)
          v = (v << 8) | data[i];
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
        for (int i = 0; i < 8; i++)
          v = (v << 8) | data[i];
        double d;
        memcpy(&d, &v, sizeof(double));
        value = static_cast<float>(d);
        return true;
      }
      break;
    case DLMS_TYPE_ENUM:
    case DLMS_TYPE_BOOLEAN:
      if (avail >= 1) {
        value = static_cast<float>(data[0]);
        return true;
      }
      break;
    case DLMS_TYPE_OCTET_STRING: {
      if (avail < 1)
        break;
      uint8_t slen = data[0];
      if (avail < 1u + slen)
        break;
      const uint8_t *p = &data[1];
      is_text = true;
      if (slen == 12) {
        snprintf(text_buf, text_buf_size, "%04d-%02d-%02d %02d:%02d:%02d", be16(p), p[2], p[3], p[5], p[6], p[7]);
      } else if (slen == 5) {
        snprintf(text_buf, text_buf_size, "%04d-%02d-%02d", be16(p), p[2], p[3]);
      } else if (slen == 4) {
        snprintf(text_buf, text_buf_size, "%02d:%02d:%02d", p[0], p[1], p[2]);
      } else {
        uint16_t text_pos = 0;
        for (uint8_t i = 0; i < slen && text_pos + 2 < text_buf_size; i++) {
          text_pos += snprintf(text_buf + text_pos, text_buf_size - text_pos, "%02X", p[i]);
        }
      }
      return true;
    }
    case DLMS_TYPE_VISIBLE_STRING:
    case DLMS_TYPE_UTF8_STRING: {
      if (avail < 1)
        break;
      uint8_t slen = data[0];
      if (avail < 1u + slen)
        break;
      is_text = true;
      uint8_t copy_len = slen;
      if (copy_len >= text_buf_size)
        copy_len = text_buf_size - 1;
      memcpy(text_buf, &data[1], copy_len);
      text_buf[copy_len] = '\0';
      return true;
    }
    default:
      break;
  }

  return false;
}

bool dlms_parse_get_response(const char *log_tag, const uint8_t *data, uint16_t len, float &value, char *text_buf,
                             uint16_t text_buf_size, bool &is_text) {
  is_text = false;

  if (len < 3 || data[0] != DLMS_TAG_GET_RESPONSE) {
    ESP_LOGW(log_tag, "Not a GET.response (tag=0x%02X)", data[0]);
    return false;
  }

  uint16_t pos = 3;
  if (pos >= len)
    return false;

  uint8_t choice = data[pos++];
  if (choice == 0x01) {
    if (pos < len)
      ESP_LOGW(log_tag, "GET.response error: access-result=%d", data[pos]);
    return false;
  }
  if (choice != 0x00) {
    ESP_LOGW(log_tag, "GET.response unexpected choice=%d", choice);
    return false;
  }

  if (pos >= len)
    return false;
  uint8_t type_tag = data[pos++];

  if (!dlms_decode_data_element(type_tag, &data[pos], len - pos, value, text_buf, text_buf_size, is_text)) {
    ESP_LOGD(log_tag, "GET.response: failed to decode DLMS type 0x%02X", type_tag);
    return false;
  }
  return true;
}

// ============================================================================
// DLMS Security — AES-128-GCM (IEC 62056-5-3, Security Suite 0)
// ============================================================================

bool dlms_aes_gcm_crypt(const char *log_tag, bool encrypt, const uint8_t *key, const uint8_t *nonce,
                        const uint8_t *aad, uint16_t aad_len, const uint8_t *input, uint16_t len, uint8_t *output,
                        uint8_t *tag_buf, uint8_t tag_len) {
  mbedtls_gcm_context gcm_ctx;
  mbedtls_gcm_init(&gcm_ctx);

  int ret = mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
  if (ret != 0) {
    ESP_LOGE(log_tag, "GCM setkey failed: %d", ret);
    mbedtls_gcm_free(&gcm_ctx);
    return false;
  }

  int mode = encrypt ? MBEDTLS_GCM_ENCRYPT : MBEDTLS_GCM_DECRYPT;
  ret = mbedtls_gcm_starts(&gcm_ctx, mode, nonce, 12);
  if (ret != 0) {
    ESP_LOGE(log_tag, "GCM starts failed: %d", ret);
    mbedtls_gcm_free(&gcm_ctx);
    return false;
  }

  if (aad != nullptr && aad_len > 0) {
    ret = mbedtls_gcm_update_ad(&gcm_ctx, aad, aad_len);
    if (ret != 0) {
      ESP_LOGE(log_tag, "GCM update_ad failed: %d", ret);
      mbedtls_gcm_free(&gcm_ctx);
      return false;
    }
  }

  size_t outlen = 0;
  ret = mbedtls_gcm_update(&gcm_ctx, input, len, output, len, &outlen);
  if (ret != 0) {
    ESP_LOGE(log_tag, "GCM %s failed: %d", encrypt ? "encrypt" : "decrypt", ret);
    mbedtls_gcm_free(&gcm_ctx);
    return false;
  }

  uint8_t computed_tag[16];
  size_t finish_outlen = 0;
  ret = mbedtls_gcm_finish(&gcm_ctx, nullptr, 0, &finish_outlen, computed_tag, sizeof(computed_tag));
  mbedtls_gcm_free(&gcm_ctx);
  if (ret != 0) {
    ESP_LOGE(log_tag, "GCM finish failed: %d", ret);
    return false;
  }

  if (tag_buf != nullptr && tag_len > 0) {
    if (encrypt) {
      memcpy(tag_buf, computed_tag, tag_len);
    } else {
      uint8_t diff = 0;
      for (uint8_t i = 0; i < tag_len; i++) {
        diff |= computed_tag[i] ^ tag_buf[i];
      }
      if (diff != 0) {
        ESP_LOGW(log_tag, "GCM auth tag mismatch — frame may be corrupted or tampered");
        return false;
      }
    }
  }

  return true;
}

uint16_t dlms_encrypt(const char *log_tag, const std::array<uint8_t, 16> &key, const std::array<uint8_t, 8> &system_title,
                      uint32_t invocation_counter, const uint8_t *apdu, uint16_t apdu_len, uint8_t *out) {
  uint16_t pos = 0;
  out[pos++] = DLMS_SC_ENC_COMP;
  out[pos++] = 0x00;

  out[pos++] = (invocation_counter >> 24) & 0xFF;
  out[pos++] = (invocation_counter >> 16) & 0xFF;
  out[pos++] = (invocation_counter >> 8) & 0xFF;
  out[pos++] = invocation_counter & 0xFF;

  uint8_t nonce[12];
  memcpy(nonce, system_title.data(), 8);
  nonce[8] = (invocation_counter >> 24) & 0xFF;
  nonce[9] = (invocation_counter >> 16) & 0xFF;
  nonce[10] = (invocation_counter >> 8) & 0xFF;
  nonce[11] = invocation_counter & 0xFF;

  uint8_t sc_aad = DLMS_SC_ENC_COMP;
  if (!dlms_aes_gcm_crypt(log_tag, true, key.data(), nonce, &sc_aad, 1, apdu, apdu_len, &out[pos])) {
    return 0;
  }
  pos += apdu_len;

  return pos;
}

// ============================================================================
// GZIP / DEFLATE decompression (mirrors firmware gzip_parse_header @ 0x1C682)
// ============================================================================

/// Strip GZIP header (RFC 1952) and return offset to raw DEFLATE data.
/// Returns 0 on error, or the number of header bytes consumed.
static uint16_t dlms_gzip_strip_header(const char *log_tag, const uint8_t *data, uint16_t len) {
  if (len < 10) {
    ESP_LOGW(log_tag, "GZIP header too short: %d", len);
    return 0;
  }

  // Magic: 1F 8B, Method: 08 (deflate)
  if (data[0] != 0x1F || data[1] != 0x8B || data[2] != 0x08) {
    ESP_LOGW(log_tag, "Not a GZIP stream (magic=%02X%02X method=%02X)", data[0], data[1], data[2]);
    return 0;
  }

  uint8_t flags = data[3];
  if (flags & 0xE0) {
    ESP_LOGW(log_tag, "GZIP reserved flags set: 0x%02X", flags);
    return 0;
  }

  // Skip: FLG(1) + MTIME(4) + XFL(1) + OS(1) = 7 bytes after magic+method
  uint16_t pos = 10;

  // FEXTRA (bit 2): 2-byte length + extra data
  if (flags & 0x04) {
    if (pos + 2 > len)
      return 0;
    uint16_t xlen = data[pos] | (data[pos + 1] << 8);
    pos += 2 + xlen;
  }

  // FNAME (bit 3): NUL-terminated string
  if (flags & 0x08) {
    while (pos < len && data[pos] != 0)
      pos++;
    pos++;  // skip NUL
  }

  // FCOMMENT (bit 4): NUL-terminated string
  if (flags & 0x10) {
    while (pos < len && data[pos] != 0)
      pos++;
    pos++;  // skip NUL
  }

  // FHCRC (bit 1): 2-byte header CRC
  if (flags & 0x02)
    pos += 2;

  if (pos >= len) {
    ESP_LOGW(log_tag, "GZIP header exceeds data length");
    return 0;
  }

  return pos;
}

/// Decompress raw DEFLATE data into apdu_out. Returns decompressed length, 0 on error.
static uint16_t dlms_inflate(const char *log_tag, const uint8_t *deflate_data, uint16_t deflate_len,
                             uint8_t *out, uint16_t out_size) {
  size_t result = tinfl_decompress_mem_to_mem(out, out_size, deflate_data, deflate_len, 0);
  if (result == TINFL_DECOMPRESS_MEM_TO_MEM_FAILED) {
    ESP_LOGW(log_tag, "DEFLATE decompression failed (in=%d bytes)", deflate_len);
    return 0;
  }
  if (result > out_size) {
    ESP_LOGW(log_tag, "Decompressed size %u exceeds buffer %u", (unsigned) result, out_size);
    return 0;
  }
  return static_cast<uint16_t>(result);
}

// ============================================================================
// DLMS Unwrap — decrypt + decompress (full security layer)
// ============================================================================

uint16_t dlms_decrypt(const char *log_tag, const std::array<uint8_t, 16> &key, const uint8_t meter_system_title[8],
                      const uint8_t *enc_payload, uint16_t enc_len, uint8_t *apdu_out) {
  if (enc_len < 7) {
    ESP_LOGW(log_tag, "Encrypted payload too short: %d", enc_len);
    return 0;
  }

  uint16_t pos = 0;
  uint8_t sc = enc_payload[pos++];
  pos++;  // skip pad byte

  if (!(sc & DLMS_SC_ENCRYPTED)) {
    // No encryption — pass through plaintext (may still be compressed)
    uint16_t plain_len = enc_len - pos;
    if (sc & DLMS_SC_COMPRESSED) {
      uint16_t hdr_len = dlms_gzip_strip_header(log_tag, &enc_payload[pos], plain_len);
      if (hdr_len == 0)
        return 0;
      uint16_t deflate_len = plain_len - hdr_len - 8;  // strip gzip trailer (CRC32 + ISIZE)
      return dlms_inflate(log_tag, &enc_payload[pos + hdr_len], deflate_len, apdu_out, MAX_APDU_DECOMPRESSED);
    }
    memcpy(apdu_out, &enc_payload[pos], plain_len);
    return plain_len;
  }

  uint32_t ic = (static_cast<uint32_t>(enc_payload[pos]) << 24) | (static_cast<uint32_t>(enc_payload[pos + 1]) << 16) |
                (static_cast<uint32_t>(enc_payload[pos + 2]) << 8) | enc_payload[pos + 3];
  pos += 4;

  uint8_t nonce[12];
  memcpy(nonce, meter_system_title, 8);
  nonce[8] = (ic >> 24) & 0xFF;
  nonce[9] = (ic >> 16) & 0xFF;
  nonce[10] = (ic >> 8) & 0xFF;
  nonce[11] = ic & 0xFF;

  uint16_t cipher_len = enc_len - pos;
  uint8_t *tag_ptr = nullptr;
  uint8_t tag_len = 0;
  if (sc & DLMS_SC_AUTHENTICATED) {
    if (cipher_len < 12)
      return 0;
    cipher_len -= 12;
    tag_ptr = const_cast<uint8_t *>(&enc_payload[pos + cipher_len]);
    tag_len = 12;
  }

  // Decrypt into apdu_out (used as temp buffer if decompression follows)
  if (!dlms_aes_gcm_crypt(log_tag, false, key.data(), nonce, &sc, 1, &enc_payload[pos], cipher_len, apdu_out, tag_ptr,
                           tag_len)) {
    return 0;
  }

  ESP_LOGD(log_tag, "Decrypted %d bytes, SC=0x%02X IC=0x%08X", cipher_len, sc, ic);

  // Decompress if SC compressed bit is set
  if (sc & DLMS_SC_COMPRESSED) {
    uint16_t hdr_len = dlms_gzip_strip_header(log_tag, apdu_out, cipher_len);
    if (hdr_len == 0)
      return 0;

    // Raw DEFLATE sits between gzip header and 8-byte trailer (CRC32 + ISIZE)
    uint16_t deflate_len = cipher_len - hdr_len;
    if (deflate_len > 8)
      deflate_len -= 8;  // strip gzip trailer

    // Need a temp buffer — can't inflate in-place
    uint8_t *decomp_buf = static_cast<uint8_t *>(malloc(MAX_APDU_DECOMPRESSED));
    if (decomp_buf == nullptr) {
      ESP_LOGE(log_tag, "Failed to allocate decompression buffer");
      return 0;
    }

    uint16_t decomp_len = dlms_inflate(log_tag, &apdu_out[hdr_len], deflate_len, decomp_buf, MAX_APDU_DECOMPRESSED);
    if (decomp_len == 0) {
      free(decomp_buf);
      return 0;
    }

    memcpy(apdu_out, decomp_buf, decomp_len);
    free(decomp_buf);

    ESP_LOGD(log_tag, "Decompressed %d -> %d bytes", cipher_len, decomp_len);
    return decomp_len;
  }

  return cipher_len;
}

}  // namespace esphome::nartis_wmbus
