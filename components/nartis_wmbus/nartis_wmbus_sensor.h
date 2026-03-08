#pragma once

#include "esphome/components/sensor/sensor.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <cstdint>
#include <cstring>
#include <string>

namespace esphome::nartis_wmbus {

static constexpr uint8_t MAX_SENSOR_TRIES = 3;

enum SensorType : uint8_t { SENSOR_NUMERIC, SENSOR_TEXT };

class NartisWmbusSensorBase {
 public:
  virtual ~NartisWmbusSensorBase() = default;
  virtual SensorType get_type() const = 0;
  virtual void publish() = 0;

  void set_obis_code(const char *code) { this->obis_code_ = code; }
  const std::string &get_obis_code() const { return this->obis_code_; }

  void set_class_id(uint16_t class_id) { this->class_id_ = class_id; }
  uint16_t get_class_id() const { return this->class_id_; }

  void set_attribute(uint8_t attr) { this->attribute_ = attr; }
  uint8_t get_attribute() const { return this->attribute_; }

  void parse_obis_bytes(uint8_t out[6]) const {
    // Parse "A.B.C.D.E.F" string into 6 bytes
    uint8_t idx = 0;
    uint16_t val = 0;
    for (char c : this->obis_code_) {
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
    this->has_value_ = false;
    this->tries_ = 0;
  }

  bool has_value() const { return this->has_value_; }
  bool is_failed() const { return this->tries_ >= MAX_SENSOR_TRIES; }

  void record_failure() {
    if (this->tries_ < MAX_SENSOR_TRIES)
      this->tries_++;
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
  void publish() override { publish_state(this->value_); }

  void set_value(float value) {
    this->value_ = value;
    this->has_value_ = true;
    this->tries_ = 0;
  }

 protected:
  float value_{0.0f};
};

#ifdef USE_TEXT_SENSOR
class NartisWmbusTextSensor : public NartisWmbusSensorBase, public text_sensor::TextSensor {
 public:
  SensorType get_type() const override { return SENSOR_TEXT; }
  void publish() override { publish_state(this->value_); }

  void set_value(const char *value) {
    this->value_ = value;
    this->has_value_ = true;
    this->tries_ = 0;
  }

 protected:
  std::string value_;
};
#endif

}  // namespace esphome::nartis_wmbus
