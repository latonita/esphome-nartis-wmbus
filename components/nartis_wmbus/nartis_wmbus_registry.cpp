#include "nartis_wmbus_registry.h"

#include <cstdio>
#include <algorithm>
#include <cstring>

namespace esphome::nartis_wmbus {

void SensorRegistry::add(NartisWmbusSensorBase *sensor) {
  this->sensors_.push_back({sensor->get_obis_code().c_str(), sensor});
}

void SensorRegistry::prepare_requests() {
  std::sort(this->sensors_.begin(), this->sensors_.end(),
            [](const SensorEntry &a, const SensorEntry &b) { return strcmp(a.obis_code, b.obis_code) < 0; });
}

void SensorRegistry::reset_all() {
  for (auto &entry : this->sensors_) {
    entry.sensor->reset();
  }
}

void SensorRegistry::publish_ready() {
  for (auto &entry : this->sensors_) {
    if (entry.sensor->has_value()) {
      entry.sensor->publish();
    }
  }
}

NartisWmbusSensorBase *SensorRegistry::current_sensor() const {
  if (!this->has_current_request()) {
    return nullptr;
  }
  return this->sensors_[this->request_idx_].sensor;
}

void SensorRegistry::advance_request() {
  if (!this->has_current_request()) {
    return;
  }

  const char *obis_code = this->sensors_[this->request_idx_].obis_code;
  while (this->request_idx_ < this->sensors_.size() && strcmp(this->sensors_[this->request_idx_].obis_code, obis_code) == 0) {
    ++this->request_idx_;
  }
}

void SensorRegistry::mark_current_failure() {
  if (!this->has_current_request()) {
    return;
  }

  const char *obis_code = this->sensors_[this->request_idx_].obis_code;
  for (auto &entry : this->sensors_) {
    if (strcmp(entry.obis_code, obis_code) == 0) {
      entry.sensor->record_failure();
    }
  }
}

void SensorRegistry::apply_current_value(float value) {
  if (!this->has_current_request()) {
    return;
  }

  const char *obis_code = this->sensors_[this->request_idx_].obis_code;
  for (auto &entry : this->sensors_) {
    if (strcmp(entry.obis_code, obis_code) != 0) {
      continue;
    }
    if (entry.sensor->get_type() == SENSOR_NUMERIC) {
      static_cast<NartisWmbusSensor *>(entry.sensor)->set_value(value);
#ifdef USE_TEXT_SENSOR
    } else if (entry.sensor->get_type() == SENSOR_TEXT) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%.3f", value);
      static_cast<NartisWmbusTextSensor *>(entry.sensor)->set_value(buf);
#endif
    }
  }
}

void SensorRegistry::apply_current_text(const char *value) {
  if (!this->has_current_request()) {
    return;
  }

  const char *obis_code = this->sensors_[this->request_idx_].obis_code;
  for (auto &entry : this->sensors_) {
    if (strcmp(entry.obis_code, obis_code) != 0) {
      continue;
    }
#ifdef USE_TEXT_SENSOR
    if (entry.sensor->get_type() == SENSOR_TEXT) {
      static_cast<NartisWmbusTextSensor *>(entry.sensor)->set_value(value);
    }
#endif
  }
}

}  // namespace esphome::nartis_wmbus
