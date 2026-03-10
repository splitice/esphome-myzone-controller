#include "myzone.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace myzone {

static const char *const TAG = "myzone";
static const uint8_t REQUEST_STATE = 0xC0;
static const uint8_t ZONE_COUNT = 6;
static const uint8_t ZONE_MASK_ALL = (1 << ZONE_COUNT) - 1;
static const uint8_t ZONE_RESYNC_INDEX = 0;
static const uint32_t STATE_REQUEST_INTERVAL_MS = 10000;
static const char *const ZONE_MASK_PREF_KEY = "myzone_zone_mask";

void MyZoneSwitch::write_state(bool state) { this->parent_->toggle_zone(this->zone_index_, state); }

void MyZoneSwitch::apply_state_from_controller(bool state) { this->publish_state(state); }

void MyZoneController::setup() {
  if (this->rse_pin_ != nullptr) {
    this->rse_pin_->setup();
    this->rse_pin_->digital_write(true);
  }

  this->zone_state_pref_ = global_preferences->make_preference<uint8_t>(fnv1_hash(ZONE_MASK_PREF_KEY));
  this->load_zone_mask_();
  this->apply_zone_mask_(this->zone_mask_, false);
  this->request_state_();
}

void MyZoneController::loop() {
  while (this->available() > 0) {
    uint8_t value = 0;
    if (!this->read_byte(&value)) {
      break;
    }
    if (value == REQUEST_STATE) {
      continue;
    }
    uint8_t new_mask = value & ZONE_MASK_ALL;
    this->apply_zone_mask_(new_mask, true);
  }

  if (millis() - this->last_state_request_ms_ >= STATE_REQUEST_INTERVAL_MS) {
    this->request_state_();
  }
}

void MyZoneController::dump_config() {
  ESP_LOGCONFIG(TAG, "MyZone Controller:");
  ESP_LOGCONFIG(TAG, "  Request byte: 0x%02X", REQUEST_STATE);
  if (this->rse_pin_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  RSE (TX enable): configured (active-low)");
  } else {
    ESP_LOGCONFIG(TAG, "  RSE (TX enable): not configured");
  }
}

float MyZoneController::get_setup_priority() const { return setup_priority::DATA; }

void MyZoneController::set_rse_pin(GPIOPin *pin) { this->rse_pin_ = pin; }

void MyZoneController::set_zone_switch(uint8_t index, MyZoneSwitch *zone_switch) {
  if (index >= ZONE_COUNT) {
    return;
  }
  this->zone_switches_[index] = zone_switch;
}

void MyZoneController::toggle_zone(uint8_t index, bool state) {
  if (index >= ZONE_COUNT) {
    return;
  }

  // Zone ZONE_RESYNC_INDEX (zone 1) is always allowed for resync;
  // other zones are skipped if they are not configured in the controller
  if (index != ZONE_RESYNC_INDEX && this->zone_switches_[index] == nullptr) {
    return;
  }

  const uint8_t zone_code = 1 << index;
  const bool current_state = (this->zone_mask_ & zone_code) != 0;
  if (current_state == state) {
    this->apply_zone_mask_(this->zone_mask_, false);
    return;
  }

  ESP_LOGI(TAG, "Changing zone %d to %s", index + 1, state ? "enabled" : "disabled");
  this->send_command_(zone_code);
  this->request_state_();
}

void MyZoneController::request_state_() {
  this->send_command_(REQUEST_STATE);
  this->last_state_request_ms_ = millis();
}

void MyZoneController::send_command_(uint8_t command) {
  if (this->rse_pin_ != nullptr) {
    this->rse_pin_->digital_write(false);
  }

  this->write_byte(command);
  this->flush();

  if (this->rse_pin_ != nullptr) {
    this->rse_pin_->digital_write(true);
  }
}

void MyZoneController::apply_zone_mask_(uint8_t mask, bool persist) {
  uint8_t new_mask = mask & ZONE_MASK_ALL;
  bool state_changed = (new_mask != this->zone_mask_);
  this->zone_mask_ = new_mask;
  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    auto *zone = this->zone_switches_[i];
    if (zone == nullptr) {
      continue;
    }
    zone->apply_state_from_controller((this->zone_mask_ & (1 << i)) != 0);
  }
  if (persist) {
    if (state_changed) {
      char state_str[ZONE_COUNT * 2];
      for (uint8_t i = 0; i < ZONE_COUNT; i++) {
        state_str[i * 2] = ((this->zone_mask_ & (1 << i)) ? '1' : '0');
        state_str[i * 2 + 1] = (i < ZONE_COUNT - 1) ? ':' : '\0';
      }
      ESP_LOGI(TAG, "New state %s", state_str);
    }
    this->save_zone_mask_();
  }
}

void MyZoneController::save_zone_mask_() { this->zone_state_pref_.save(&this->zone_mask_); }

void MyZoneController::load_zone_mask_() {
  if (!this->zone_state_pref_.load(&this->zone_mask_)) {
    this->zone_mask_ = 0;
  }
}

}  // namespace myzone
}  // namespace esphome
