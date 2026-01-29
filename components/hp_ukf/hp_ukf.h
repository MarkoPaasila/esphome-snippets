#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "hp_ukf_ukf.h"

namespace esphome {
namespace hp_ukf {

class HpUkfComponent : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_inlet_temperature_sensor(sensor::Sensor *s) { inlet_temperature_ = s; }
  void set_inlet_humidity_sensor(sensor::Sensor *s) { inlet_humidity_ = s; }
  void set_outlet_temperature_sensor(sensor::Sensor *s) { outlet_temperature_ = s; }
  void set_outlet_humidity_sensor(sensor::Sensor *s) { outlet_humidity_ = s; }
  void set_track_temperature_derivatives(bool v) { track_derivatives_ = v; }

  void set_filtered_inlet_temperature_sensor(sensor::Sensor *s) { filtered_inlet_temperature_ = s; }
  void set_filtered_inlet_humidity_sensor(sensor::Sensor *s) { filtered_inlet_humidity_ = s; }
  void set_filtered_outlet_temperature_sensor(sensor::Sensor *s) { filtered_outlet_temperature_ = s; }
  void set_filtered_outlet_humidity_sensor(sensor::Sensor *s) { filtered_outlet_humidity_ = s; }
  void set_filtered_inlet_temperature_derivative_sensor(sensor::Sensor *s) {
    filtered_inlet_temperature_derivative_ = s;
  }
  void set_filtered_outlet_temperature_derivative_sensor(sensor::Sensor *s) {
    filtered_outlet_temperature_derivative_ = s;
  }
  void set_filtered_inlet_humidity_derivative_sensor(sensor::Sensor *s) {
    filtered_inlet_humidity_derivative_ = s;
  }
  void set_filtered_outlet_humidity_derivative_sensor(sensor::Sensor *s) {
    filtered_outlet_humidity_derivative_ = s;
  }

 protected:
  sensor::Sensor *inlet_temperature_{nullptr};
  sensor::Sensor *inlet_humidity_{nullptr};
  sensor::Sensor *outlet_temperature_{nullptr};
  sensor::Sensor *outlet_humidity_{nullptr};
  bool track_derivatives_{true};

  sensor::Sensor *filtered_inlet_temperature_{nullptr};
  sensor::Sensor *filtered_inlet_humidity_{nullptr};
  sensor::Sensor *filtered_outlet_temperature_{nullptr};
  sensor::Sensor *filtered_outlet_humidity_{nullptr};
  sensor::Sensor *filtered_inlet_temperature_derivative_{nullptr};
  sensor::Sensor *filtered_outlet_temperature_derivative_{nullptr};
  sensor::Sensor *filtered_inlet_humidity_derivative_{nullptr};
  sensor::Sensor *filtered_outlet_humidity_derivative_{nullptr};

  HpUkfFilter filter_;
  uint32_t last_update_ms_{0};
  bool initialized_{false};
};

}  // namespace hp_ukf
}  // namespace esphome
