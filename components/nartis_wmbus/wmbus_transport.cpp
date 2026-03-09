#include "wmbus_transport.h"

#include "esphome/core/log.h"

#include <cstring>

namespace esphome::nartis_wmbus {

static const uint16_t CRC16_EN13757_TABLE[256] = {
    0x0000, 0x3D65, 0x7ACA, 0x47AF, 0xF594, 0xC8F1, 0x8F5E, 0xB23B, 0xD64D, 0xEB28, 0xAC87, 0x91E2, 0x23D9, 0x1EBC,
    0x5913, 0x6476, 0x91FF, 0xAC9A, 0xEB35, 0xD650, 0x646B, 0x590E, 0x1EA1, 0x23C4, 0x47B2, 0x7AD7, 0x3D78, 0x001D,
    0xB226, 0x8F43, 0xC8EC, 0xF589, 0x1E9B, 0x23FE, 0x6451, 0x5934, 0xEB0F, 0xD66A, 0x91C5, 0xACA0, 0xC8D6, 0xF5B3,
    0xB21C, 0x8F79, 0x3D42, 0x0027, 0x4788, 0x7AED, 0x8F64, 0xB201, 0xF5AE, 0xC8CB, 0x7AF0, 0x4795, 0x003A, 0x3D5F,
    0x5929, 0x644C, 0x23E3, 0x1E86, 0xACBD, 0x91D8, 0xD677, 0xEB12, 0x3D36, 0x0053, 0x47FC, 0x7A99, 0xC8A2, 0xF5C7,
    0xB268, 0x8F0D, 0xEB7B, 0xD61E, 0x91B1, 0xACD4, 0x1EEF, 0x238A, 0x6425, 0x5940, 0xACC9, 0x91AC, 0xD603, 0xEB66,
    0x595D, 0x6438, 0x2397, 0x1EF2, 0x7A84, 0x47E1, 0x004E, 0x3D2B, 0x8F10, 0xB275, 0xF5DA, 0xC8BF, 0x23AD, 0x1EC8,
    0x5967, 0x6402, 0xD639, 0xEB5C, 0xACF3, 0x9196, 0xF5E0, 0xC885, 0x8F2A, 0xB24F, 0x0074, 0x3D11, 0x7ABE, 0x47DB,
    0xB252, 0x8F37, 0xC898, 0xF5FD, 0x47C6, 0x7AA3, 0x3D0C, 0x0069, 0x641F, 0x597A, 0x1ED5, 0x23B0, 0x918B, 0xACEE,
    0xEB41, 0xD624, 0x7A6C, 0x4709, 0x00A6, 0x3DC3, 0x8FF8, 0xB29D, 0xF532, 0xC857, 0xAC21, 0x9144, 0xD6EB, 0xEB8E,
    0x59B5, 0x64D0, 0x237F, 0x1E1A, 0xEB93, 0xD6F6, 0x9159, 0xAC3C, 0x1E07, 0x2362, 0x64CD, 0x59A8, 0x3DDE, 0x00BB,
    0x4714, 0x7A71, 0xC84A, 0xF52F, 0xB280, 0x8FE5, 0x64F7, 0x5992, 0x1E3D, 0x2358, 0x9163, 0xAC06, 0xEBA9, 0xD6CC,
    0xB2BA, 0x8FDF, 0xC870, 0xF515, 0x472E, 0x7A4B, 0x3DE4, 0x0081, 0xF508, 0xC86D, 0x8FC2, 0xB2A7, 0x009C, 0x3DF9,
    0x7A56, 0x4733, 0x2345, 0x1E20, 0x598F, 0x64EA, 0xD6D1, 0xEBB4, 0xAC1B, 0x917E, 0x475A, 0x7A3F, 0x3D90, 0x00F5,
    0xB2CE, 0x8FAB, 0xC804, 0xF561, 0x9117, 0xAC72, 0xEBDD, 0xD6B8, 0x6483, 0x59E6, 0x1E49, 0x232C, 0xD6A5, 0xEBC0,
    0xAC6F, 0x910A, 0x2331, 0x1E54, 0x59FB, 0x649E, 0x00E8, 0x3D8D, 0x7A22, 0x4747, 0xF57C, 0xC819, 0x8FB6, 0xB2D3,
    0x59C1, 0x64A4, 0x230B, 0x1E6E, 0xAC55, 0x9130, 0xD69F, 0xEBFA, 0x8F8C, 0xB2E9, 0xF546, 0xC823, 0x7A18, 0x477D,
    0x00D2, 0x3DB7, 0xC83E, 0xF55B, 0xB2F4, 0x8F91, 0x3DAA, 0x00CF, 0x4760, 0x7A05, 0x1E73, 0x2316, 0x64B9, 0x59DC,
    0xEBE7, 0xD682, 0x912D, 0xAC48,
};

uint16_t wmbus_crc16_en13757(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0x0000;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t idx = static_cast<uint8_t>(crc >> 8) ^ data[i];
    crc = (crc << 8) ^ CRC16_EN13757_TABLE[idx];
  }
  return ~crc & 0xFFFF;
}

uint16_t wmbus_frame_build(uint8_t c_field, const uint8_t *payload, uint16_t pay_len, uint8_t access_nr, uint8_t *out) {
  uint8_t raw[MAX_APDU_SIZE + 30];
  uint16_t raw_len = 0;

  ESP_LOGV("nartis_wmbus.wmbus", "frame_build: C=0x%02X pay_len=%d access_nr=%d", c_field, pay_len, access_nr);

  raw[raw_len++] = 0;
  raw[raw_len++] = c_field;
  raw[raw_len++] = OUR_MANUFACTURER & 0xFF;
  raw[raw_len++] = (OUR_MANUFACTURER >> 8) & 0xFF;

  memcpy(&raw[raw_len], OUR_ADDRESS, 4);
  raw_len += 4;
  raw[raw_len++] = OUR_VERSION;
  raw[raw_len++] = OUR_DEVICE_TYPE;

  raw[raw_len++] = WMBUS_CI_TPL_SHORT;
  raw[raw_len++] = c_field;
  raw[raw_len++] = WMBUS_CI_TPL_SHORT;
  raw[raw_len++] = access_nr;
  raw[raw_len++] = 0x00;

  memcpy(&raw[raw_len], payload, pay_len);
  raw_len += pay_len;
  raw[0] = raw_len - 1;

  uint16_t out_len = 0;
  uint16_t pos = 0;
  uint16_t first_block = WMBUS_FIRST_BLOCK;
  if (first_block > raw_len)
    first_block = raw_len;

  memcpy(&out[out_len], &raw[pos], first_block);
  out_len += first_block;
  pos += first_block;

  uint16_t crc = wmbus_crc16_en13757(&raw[0], first_block);
  out[out_len++] = (crc >> 8) & 0xFF;
  out[out_len++] = crc & 0xFF;

  while (pos < raw_len) {
    uint16_t block_len = raw_len - pos;
    if (block_len > WMBUS_BLOCK_SIZE)
      block_len = WMBUS_BLOCK_SIZE;

    memcpy(&out[out_len], &raw[pos], block_len);
    out_len += block_len;

    crc = wmbus_crc16_en13757(&raw[pos], block_len);
    out[out_len++] = (crc >> 8) & 0xFF;
    out[out_len++] = crc & 0xFF;

    pos += block_len;
  }

  ESP_LOGV("nartis_wmbus.wmbus", "frame_build: raw=%d -> interleaved=%d bytes (L=%d)", raw_len, out_len, raw[0]);
  return out_len;
}

uint16_t wmbus_frame_parse(const char *log_tag, const uint8_t *frame, uint16_t frame_len, uint8_t *out) {
  ESP_LOGVV(log_tag, "wmbus_frame_parse: frame_len=%d", frame_len);
  uint16_t in_pos = 0;
  uint16_t out_len = 0;

  if (in_pos + WMBUS_FIRST_BLOCK + 2 > frame_len)
    return 0;

  uint16_t crc_calc = wmbus_crc16_en13757(&frame[in_pos], WMBUS_FIRST_BLOCK);
  uint16_t crc_recv = (frame[in_pos + WMBUS_FIRST_BLOCK] << 8) | frame[in_pos + WMBUS_FIRST_BLOCK + 1];
  if (crc_calc != crc_recv) {
    ESP_LOGW(log_tag, "CRC error in first block: calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
    return 0;
  }

  memcpy(&out[out_len], &frame[in_pos], WMBUS_FIRST_BLOCK);
  out_len += WMBUS_FIRST_BLOCK;
  in_pos += WMBUS_FIRST_BLOCK + 2;

  uint8_t l_field = out[0];
  uint16_t expected_data = l_field + 1;

  while (out_len < expected_data && in_pos < frame_len) {
    uint16_t remaining = expected_data - out_len;
    uint16_t block_len = remaining > WMBUS_BLOCK_SIZE ? WMBUS_BLOCK_SIZE : remaining;

    if (in_pos + block_len + 2 > frame_len) {
      ESP_LOGVV(log_tag, "wmbus_frame_parse: truncated data block at pos %d (need %d, have %d)",
                in_pos, block_len + 2, frame_len - in_pos);
      return 0;
    }

    crc_calc = wmbus_crc16_en13757(&frame[in_pos], block_len);
    crc_recv = (frame[in_pos + block_len] << 8) | frame[in_pos + block_len + 1];
    if (crc_calc != crc_recv) {
      ESP_LOGW(log_tag, "CRC error in data block at pos %d: calc=0x%04X recv=0x%04X (block_len=%d)", in_pos, crc_calc, crc_recv, block_len);
      return 0;
    }

    memcpy(&out[out_len], &frame[in_pos], block_len);
    out_len += block_len;
    in_pos += block_len + 2;
  }

  ESP_LOGVV(log_tag, "wmbus_frame_parse: OK, L=%d stripped=%d", l_field, out_len);
  return out_len;
}

void build_install_payload(const char *meter_id, uint8_t out[INSTALL_PAYLOAD_SIZE]) {
  if (meter_id[0] != '\0') {
    uint8_t pad = INSTALL_PAYLOAD_SIZE;
    uint8_t copy_len = strlen(meter_id);
    if (copy_len > INSTALL_PAYLOAD_SIZE)
      copy_len = INSTALL_PAYLOAD_SIZE;
    pad -= copy_len;
    memset(out, 0x30, pad);
    memcpy(out + pad, meter_id, copy_len);
    ESP_LOGV("nartis_wmbus.wmbus", "build_install_payload: meter_id='%s' pad=%d", meter_id, pad);
  } else {
    memset(out, 0x30, INSTALL_PAYLOAD_SIZE);
    ESP_LOGV("nartis_wmbus.wmbus", "build_install_payload: no meter_id, all '0' padding");
  }
}

void decode_manufacturer(uint16_t m_field, char out[4]) {
  out[0] = static_cast<char>(((m_field >> 10) & 0x1F) + 64);
  out[1] = static_cast<char>(((m_field >> 5) & 0x1F) + 64);
  out[2] = static_cast<char>((m_field & 0x1F) + 64);
  out[3] = '\0';
}

}  // namespace esphome::nartis_wmbus
