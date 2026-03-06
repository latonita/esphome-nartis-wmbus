#pragma once

#include "esphome/components/sensor/sensor.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <cstdint>
#include <cstring>
#include <string>

namespace esphome {
namespace nartis_wmbus {

static constexpr uint8_t MAX_SENSOR_TRIES = 3;

enum SensorType : uint8_t { SENSOR_NUMERIC, SENSOR_TEXT };

class NartisWmbusSensorBase {
 public:
  virtual ~NartisWmbusSensorBase() = default;
  virtual SensorType get_type() const = 0;
  virtual void publish() = 0;

  void set_obis_code(const char *code) { obis_code_ = code; }
  const std::string &get_obis_code() const { return obis_code_; }

  void set_class_id(uint16_t class_id) { class_id_ = class_id; }
  uint16_t get_class_id() const { return class_id_; }

  void set_attribute(uint8_t attr) { attribute_ = attr; }
  uint8_t get_attribute() const { return attribute_; }

  void parse_obis_bytes(uint8_t out[6]) const {
    // Parse "A.B.C.D.E.F" string into 6 bytes
    uint8_t idx = 0;
    uint16_t val = 0;
    for (char c : obis_code_) {
      if (c == '.') {
        if (idx < 6) out[idx++] = static_cast<uint8_t>(val);
        val = 0;
      } else if (c >= '0' && c <= '9') {
        val = val * 10 + (c - '0');
      }
    }
    if (idx < 6) out[idx] = static_cast<uint8_t>(val);
  }

  void reset() {
    has_value_ = false;
    tries_ = 0;
  }

  bool has_value() const { return has_value_; }
  bool is_failed() const { return tries_ >= MAX_SENSOR_TRIES; }

  void record_failure() {
    if (tries_ < MAX_SENSOR_TRIES)
      tries_++;
  }

 protected:
  std::string obis_code_;
  uint16_t class_id_{3};
  uint8_t attribute_{2};
  bool has_value_{false};
  uint8_t tries_{0};
};

class NartisWmbusSensor : public NartisWmbusSensorBase, public sensor::Sensor {
 public:
  SensorType get_type() const override { return SENSOR_NUMERIC; }
  void publish() override { publish_state(value_); }

  void set_value(float value) {
    value_ = value;
    has_value_ = true;
    tries_ = 0;
  }

 protected:
  float value_{0.0f};
};

#ifdef USE_TEXT_SENSOR
class NartisWmbusTextSensor : public NartisWmbusSensorBase, public text_sensor::TextSensor {
 public:
  SensorType get_type() const override { return SENSOR_TEXT; }
  void publish() override { publish_state(value_); }

  void set_value(const char *value) {
    value_ = value;
    has_value_ = true;
    tries_ = 0;
  }

 protected:
  std::string value_;
};
#endif

}  // namespace nartis_wmbus
}  // namespace esphome
