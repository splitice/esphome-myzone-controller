#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace myzone {

class MyZoneController;

class MyZoneSwitch : public switch_::Switch, public Parented<MyZoneController> {
 public:
  explicit MyZoneSwitch(uint8_t zone_index) : zone_index_(zone_index) {}

  void write_state(bool state) override;
  void apply_state_from_controller(bool state);

 protected:
  uint8_t zone_index_;
};

class MyZoneController : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_rse_pin(GPIOPin *pin);
  void set_zone_switch(uint8_t index, MyZoneSwitch *zone_switch);
  void toggle_zone(uint8_t index, bool state);

 protected:
  void request_state_();
  void send_command_(uint8_t command);
  void apply_zone_mask_(uint8_t mask, bool persist);
  void save_zone_mask_();
  void load_zone_mask_();
  bool should_ignore_echo_(uint8_t value);

  GPIOPin *rse_pin_{nullptr};
  MyZoneSwitch *zone_switches_[6]{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  ESPPreferenceObject zone_state_pref_;
  uint8_t zone_mask_{0};
  uint8_t pending_zone_command_echo_{0};
  uint32_t pending_zone_command_echo_ms_{0};
  uint32_t last_state_request_ms_{0};
};

}  // namespace myzone
}  // namespace esphome
