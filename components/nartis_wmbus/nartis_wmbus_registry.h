#pragma once

#include "nartis_wmbus_sensor.h"

#include <vector>

namespace esphome::nartis_wmbus {

struct SensorEntry {
  const char *obis_code;  // points into sensor's std::string member (stable)
  NartisWmbusSensorBase *sensor;
};

class SensorRegistry {
 public:
  void add(NartisWmbusSensorBase *sensor);
  void prepare_requests();

  bool empty() const { return this->sensors_.empty(); }
  size_t size() const { return this->sensors_.size(); }

  void reset_all();
  void publish_ready();

  void start_requests() { this->request_idx_ = 0; }
  bool has_current_request() const { return this->request_idx_ < this->sensors_.size(); }
  NartisWmbusSensorBase *current_sensor() const;
  void advance_request();

  void mark_current_failure();
  void apply_current_value(float value);
  void apply_current_text(const char *value);

 protected:
  std::vector<SensorEntry> sensors_;
  size_t request_idx_{0};
};

}  // namespace esphome::nartis_wmbus
