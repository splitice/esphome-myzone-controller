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
static const uint32_t COMMAND_ECHO_IGNORE_WINDOW_MS = 250;
static const uint32_t STATE_REQUEST_INTERVAL_MS = 10000;
static const uint32_t RESPONSE_WAIT_WINDOW_MS = 200;
static const char *const ZONE_MASK_PREF_KEY = "myzone_zone_mask";
static const uint8_t FRAME_LEN = 8;

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
  if (this->pending_response_command_ != 0 && !this->is_waiting_for_response_()) {
    ESP_LOGD(TAG, "Timed out waiting for response to 0x%02X", this->pending_response_command_);
    this->reset_response_parser_();
    this->pending_response_command_ = 0;
    this->flush_discarded_invalid_bytes_log_();
  }

  while (this->available() > 0) {
    uint8_t value = 0;
    if (!this->read_byte(&value)) {
      break;
    }

    if (!this->is_waiting_for_response_()) {
      this->discarded_invalid_byte_count_++;
      continue;
    }

    if (this->response_frame_pos_ == 0) {
      if (value != FRAME_LEN) {
        this->discarded_invalid_byte_count_++;
        continue;
      }
      this->response_frame_buffer_[this->response_frame_pos_++] = value;
      continue;
    }

    this->response_frame_buffer_[this->response_frame_pos_++] = value;
    if (this->response_frame_pos_ < FRAME_LEN) {
      continue;
    }

    uint8_t new_mask = 0;
    FrameValidationError error = FrameValidationError::NONE;
    uint8_t error_index = 0;
    uint8_t expected = 0;
    uint8_t actual = 0;
    const bool parsed = this->parse_zone_frame_(this->response_frame_buffer_, &new_mask, &error, &error_index, &expected, &actual);
    if (parsed) {
      this->apply_zone_mask_(new_mask, true);
      this->pending_response_command_ = 0;
      this->reset_response_parser_();
      this->flush_discarded_invalid_bytes_log_();
      continue;
    }

    ESP_LOGW(TAG,
             "Discarded invalid frame [%02X %02X %02X %02X %02X %02X %02X %02X]: %s (index=%u expected=0x%02X got=0x%02X)",
             this->response_frame_buffer_[0], this->response_frame_buffer_[1], this->response_frame_buffer_[2],
             this->response_frame_buffer_[3], this->response_frame_buffer_[4], this->response_frame_buffer_[5],
             this->response_frame_buffer_[6], this->response_frame_buffer_[7],
             this->frame_validation_error_to_string_(error), error_index, expected, actual);
    this->discarded_invalid_byte_count_ += FRAME_LEN;
    this->reset_response_parser_();
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
  this->pending_zone_command_echo_ = zone_code;
  this->pending_zone_command_echo_ms_ = millis();
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
  this->reset_response_parser_();
  this->pending_response_command_ = command;
  this->response_wait_started_ms_ = millis();

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

bool MyZoneController::should_ignore_echo_(uint8_t value) {
  if (this->pending_zone_command_echo_ == 0) {
    return false;
  }

  if (millis() - this->pending_zone_command_echo_ms_ > COMMAND_ECHO_IGNORE_WINDOW_MS) {
    this->pending_zone_command_echo_ = 0;
    return false;
  }

  if (value != this->pending_zone_command_echo_) {
    return false;
  }

  ESP_LOGD(TAG, "Ignoring echoed command byte 0x%02X", value);
  this->pending_zone_command_echo_ = 0;
  return true;
}

bool MyZoneController::is_waiting_for_response_() const {
  if (this->pending_response_command_ == 0) {
    return false;
  }
  return millis() - this->response_wait_started_ms_ <= RESPONSE_WAIT_WINDOW_MS;
}

void MyZoneController::flush_discarded_invalid_bytes_log_() {
  if (this->discarded_invalid_byte_count_ == 0) {
    return;
  }

  ESP_LOGW(TAG, "Discarded %u invalid byte(s)", this->discarded_invalid_byte_count_);
  this->discarded_invalid_byte_count_ = 0;
}

void MyZoneController::reset_response_parser_() {
  this->response_frame_pos_ = 0;
}

uint8_t MyZoneController::compute_zone_frame_checksum_(const uint8_t *frame) const {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < FRAME_LEN - 1; i++) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool MyZoneController::parse_zone_frame_(const uint8_t *frame, uint8_t *zone_mask, FrameValidationError *error,
                                         uint8_t *error_index, uint8_t *expected, uint8_t *actual) const {
  if (error != nullptr) {
    *error = FrameValidationError::NONE;
  }
  if (error_index != nullptr) {
    *error_index = 0;
  }
  if (expected != nullptr) {
    *expected = 0;
  }
  if (actual != nullptr) {
    *actual = 0;
  }

  if (frame == nullptr || zone_mask == nullptr) {
    if (error != nullptr) {
      *error = FrameValidationError::INVALID_LENGTH;
    }
    return false;
  }

  if (frame[0] != FRAME_LEN) {
    if (error != nullptr) {
      *error = FrameValidationError::INVALID_LENGTH;
    }
    if (expected != nullptr) {
      *expected = FRAME_LEN;
    }
    if (actual != nullptr) {
      *actual = frame[0];
    }
    return false;
  }

  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    const uint8_t expected_low7 = i;
    const uint8_t actual_low7 = frame[i + 1] & 0x7F;
    if (actual_low7 != expected_low7) {
      if (error != nullptr) {
        *error = FrameValidationError::INVALID_ZONE_INDEX;
      }
      if (error_index != nullptr) {
        *error_index = i;
      }
      if (expected != nullptr) {
        *expected = expected_low7;
      }
      if (actual != nullptr) {
        *actual = actual_low7;
      }
      return false;
    }
  }

  const uint8_t expected_checksum = this->compute_zone_frame_checksum_(frame);
  if (expected_checksum != frame[7]) {
    if (error != nullptr) {
      *error = FrameValidationError::CHECKSUM_MISMATCH;
    }
    if (error_index != nullptr) {
      *error_index = 7;
    }
    if (expected != nullptr) {
      *expected = expected_checksum;
    }
    if (actual != nullptr) {
      *actual = frame[7];
    }
    return false;
  }

  uint8_t mask = 0;
  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    if ((frame[i + 1] & 0x80) != 0) {
      mask |= (1u << i);
    }
  }
  *zone_mask = mask;
  return true;
}

const char *MyZoneController::frame_validation_error_to_string_(FrameValidationError error) const {
  switch (error) {
    case FrameValidationError::INVALID_LENGTH:
      return "invalid length byte";
    case FrameValidationError::INVALID_ZONE_INDEX:
      return "invalid zone index field";
    case FrameValidationError::CHECKSUM_MISMATCH:
      return "checksum mismatch";
    case FrameValidationError::NONE:
    default:
      return "unknown validation error";
  }
}

}  // namespace myzone
}  // namespace esphome
