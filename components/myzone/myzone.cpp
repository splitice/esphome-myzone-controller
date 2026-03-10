#include "myzone.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP_IDF
#include "driver/uart.h"
#include "driver/uart_select.h"
#include "hal/uart_ll.h"
#include "esphome/core/application.h"
#include "esphome/components/uart/uart_component_esp_idf.h"
#endif

namespace esphome {
namespace myzone {

static const char *const TAG = "myzone";
static const uint8_t COMMAND_FRAME_START = 0xC0;
static const uint8_t COMMAND_FRAME_END = 0xFF;
static const uint8_t REQUEST_STATE_BUTTON_CODE = 0x00;
static const uint8_t ZONE_COUNT = 6;
static const uint8_t ZONE_MASK_ALL = (1 << ZONE_COUNT) - 1;
static const uint8_t ZONE_RESYNC_INDEX = 0;
static const uint32_t COMMAND_ECHO_IGNORE_WINDOW_MS = 250;
static const uint32_t STATE_REQUEST_INTERVAL_MS = 100000;
static const uint32_t RESPONSE_WAIT_WINDOW_MS = 200;
static const uint32_t COMMAND_FRAME_TIMEOUT_MS = 50;
static const uint8_t RESPONSE_FRAME_START = 0x08;
static const uint32_t RS485_DIRECTION_SETTLE_DELAY_US = 100;
static const uint32_t UART_ERROR_LOG_THROTTLE_MS = 200;
static const char *const ZONE_MASK_PREF_KEY = "myzone_zone_mask";
static const uint8_t FRAME_LEN = 8;
static const uint8_t COMMAND_FRAME_LEN = 3;

#ifdef USE_ESP_IDF
MyZoneController *MyZoneController::uart_owner_by_num_[SOC_UART_NUM]{nullptr};
#endif

void MyZoneSwitch::write_state(bool state) { this->parent_->toggle_zone(this->zone_index_, state); }

void MyZoneSwitch::apply_state_from_controller(bool state) { this->publish_state(state); }

void MyZoneController::setup() {
  if (this->rse_pin_ != nullptr) {
    this->rse_pin_->setup();
    this->rse_pin_->pin_mode(gpio::FLAG_OUTPUT);
    this->rse_pin_->digital_write(false);
  }

  this->zone_state_pref_ = global_preferences->make_preference<uint8_t>(fnv1_hash(ZONE_MASK_PREF_KEY));
  this->load_zone_mask_();
  this->apply_zone_mask_(this->zone_mask_, false);
#ifdef USE_ESP_IDF
  this->register_esp_idf_uart_isr_callback_();
#endif
  this->request_state_();
}

void MyZoneController::loop() {
#ifdef USE_ESP_IDF
  this->log_esp_idf_uart_errors_();
#endif

  if (this->pending_response_command_ != 0 && !this->is_waiting_for_response_()) {
    ESP_LOGD(TAG, "Timed out waiting for response to 0x%02X", this->pending_response_command_);
    this->pending_response_command_ = 0;
    this->flush_discarded_non_frame_start_bytes_log_();
  }

  if (this->command_frame_pos_ > 0 && millis() - this->command_frame_started_ms_ > COMMAND_FRAME_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Resetting partial command frame after timeout");
    this->reset_command_parser_();
  }

  while (this->available()) {
    uint8_t value = 0;
    if (!this->read_byte(&value)) {
      break;
    }

    // log incoming byte for debugging
    ESP_LOGW(TAG, "Read 0x%02X", value);

    // parsing of button command
    if (this->command_frame_pos_ > 0) {
      this->command_frame_buffer_[this->command_frame_pos_++] = value;
      this->command_frame_started_ms_ = millis();
      if (this->command_frame_pos_ < COMMAND_FRAME_LEN) {
        continue;
      }

      const uint8_t button_code = this->command_frame_buffer_[1];
      const uint8_t frame_end = this->command_frame_buffer_[2];
      if (frame_end == COMMAND_FRAME_END) {
        ESP_LOGI(TAG, "Observed network command button 0x%02X", button_code);
      } else {
        ESP_LOGW(TAG, "Discarded invalid command frame [0x%02X 0x%02X 0x%02X]", this->command_frame_buffer_[0],
                 this->command_frame_buffer_[1], this->command_frame_buffer_[2]);
        this->discarded_invalid_byte_count_ += COMMAND_FRAME_LEN;
      }
      this->reset_command_parser_();
      continue;
    }

    if (this->response_frame_pos_ > 0) {
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
        if (this->pending_response_command_ != 0) {
          this->pending_response_command_ = 0;
        }
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
      this->flush_discarded_non_frame_start_bytes_log_();
      continue;
    }

    // starting a response frame
    if (this->response_frame_pos_ == 0 && this->command_frame_pos_ == 0) {
      if (value == COMMAND_FRAME_START) {
        this->flush_discarded_non_frame_start_bytes_log_();
        this->command_frame_buffer_[this->command_frame_pos_++] = value;
        this->command_frame_started_ms_ = millis();
        continue;
      }
      if (value == RESPONSE_FRAME_START) {
        this->flush_discarded_non_frame_start_bytes_log_();
        this->response_frame_buffer_[this->response_frame_pos_++] = value;
        continue;
      }

      this->discarded_non_frame_start_byte_count_++;
      ESP_LOGD(TAG, "Discarding non-protocol-start byte 0x%02X", value);
      this->reset_response_parser_();
      this->reset_command_parser_();
      
      continue;
    }

    
  }

  if (this->response_frame_pos_ == 0 && this->command_frame_pos_ == 0 && (millis() - this->last_state_request_ms_) >= STATE_REQUEST_INTERVAL_MS) {
    this->request_state_();
  }
}

void MyZoneController::dump_config() {
  ESP_LOGCONFIG(TAG, "MyZone Controller:");
  ESP_LOGCONFIG(TAG, "  Command frame: 0x%02X <button_code> 0x%02X", COMMAND_FRAME_START, COMMAND_FRAME_END);
  ESP_LOGCONFIG(TAG, "  State request button code: 0x%02X", REQUEST_STATE_BUTTON_CODE);
  if (this->rse_pin_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  RSE (TX enable): configured (active-high)");
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
}

void MyZoneController::request_state_() {
  this->send_command_(REQUEST_STATE_BUTTON_CODE);
  this->last_state_request_ms_ = millis();
}

void MyZoneController::send_command_(uint8_t command) {
  if (this->rse_pin_ != nullptr) {
    this->rse_pin_->digital_write(true);
    delayMicroseconds(RS485_DIRECTION_SETTLE_DELAY_US);
  }

  ESP_LOGW(TAG, "Sending command byte 0x%02X", command);

  this->write_byte(COMMAND_FRAME_START);
  this->write_byte(command);
  this->write_byte(COMMAND_FRAME_END);
  this->flush();

  if (this->rse_pin_ != nullptr) {
    delayMicroseconds(RS485_DIRECTION_SETTLE_DELAY_US);
  }

  this->reset_response_parser_();
  this->pending_response_command_ = command;
  this->response_wait_started_ms_ = millis();

  if (this->rse_pin_ != nullptr) {
    this->rse_pin_->digital_write(false);
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

#ifdef USE_ESP_IDF
void MyZoneController::log_esp_idf_uart_errors_() {
  if (millis() - this->last_uart_error_log_ms_ < UART_ERROR_LOG_THROTTLE_MS) {
    return;
  }

  const uint32_t parity_count = this->isr_parity_error_count_;
  if (parity_count > 0) {
    ESP_LOGW(TAG, "UART parity error(s) detected (IDF ISR): %u", parity_count);
    this->isr_parity_error_count_ = 0;
  }

  const uint32_t frame_count = this->isr_frame_error_count_;
  if (frame_count > 0) {
    ESP_LOGW(TAG, "UART frame error(s) detected (IDF ISR): %u", frame_count);
    this->isr_frame_error_count_ = 0;
  }

  const uint32_t overflow_count = this->isr_fifo_overflow_count_;
  if (overflow_count > 0) {
    ESP_LOGW(TAG, "UART FIFO overflow(s) detected (IDF ISR): %u", overflow_count);
    this->isr_fifo_overflow_count_ = 0;
  }

  if (parity_count > 0 || frame_count > 0 || overflow_count > 0) {
    this->last_uart_error_log_ms_ = millis();
  }
}

void MyZoneController::register_esp_idf_uart_isr_callback_() {
  auto *idf_parent = static_cast<uart::IDFUARTComponent *>(this->parent_);
  if (idf_parent == nullptr) {
    return;
  }

  this->uart_num_ = static_cast<int8_t>(idf_parent->get_hw_serial_number());
  if (this->uart_num_ < 0 || this->uart_num_ >= SOC_UART_NUM) {
    return;
  }

  uart_owner_by_num_[this->uart_num_] = this;
  uart_set_select_notif_callback(static_cast<uart_port_t>(this->uart_num_), &MyZoneController::uart_select_isr_callback_);

  const uint32_t intr_mask = UART_INTR_PARITY_ERR | UART_INTR_FRAM_ERR | UART_INTR_RXFIFO_OVF;
  uart_enable_intr_mask(static_cast<uart_port_t>(this->uart_num_), intr_mask);
}

void MyZoneController::uart_select_isr_callback_(uart_port_t uart_num, uart_select_notif_t uart_select_notif,
                                                 BaseType_t *task_woken) {
  if (uart_select_notif == UART_SELECT_READ_NOTIF) {
    Application::wake_loop_isrsafe(task_woken);
  }

  if (uart_num < 0 || uart_num >= SOC_UART_NUM) {
    return;
  }

  auto *owner = uart_owner_by_num_[uart_num];
  if (owner == nullptr) {
    return;
  }

  uart_dev_t *hw = UART_LL_GET_HW(uart_num);
  const uint32_t status = uart_ll_get_intsts_mask(hw);
  uint32_t clear_mask = 0;

  if ((status & UART_INTR_PARITY_ERR) != 0) {
    owner->isr_parity_error_count_++;
    clear_mask |= UART_INTR_PARITY_ERR;
  }

  if ((status & UART_INTR_FRAM_ERR) != 0) {
    owner->isr_frame_error_count_++;
    clear_mask |= UART_INTR_FRAM_ERR;
  }

  if ((status & UART_INTR_RXFIFO_OVF) != 0) {
    owner->isr_fifo_overflow_count_++;
    clear_mask |= UART_INTR_RXFIFO_OVF;
  }

  if (clear_mask != 0) {
    uart_ll_clr_intsts_mask(hw, clear_mask);
  }
}
#endif

void MyZoneController::flush_discarded_invalid_bytes_log_() {
  if (this->discarded_invalid_byte_count_ == 0) {
    return;
  }

  ESP_LOGW(TAG, "Discarded %u invalid frame byte(s)", this->discarded_invalid_byte_count_);
  this->discarded_invalid_byte_count_ = 0;
}

void MyZoneController::flush_discarded_non_frame_start_bytes_log_() {
  if (this->discarded_non_frame_start_byte_count_ == 0) {
    return;
  }

  ESP_LOGW(TAG, "Discarded %u non-protocol-start byte(s)", this->discarded_non_frame_start_byte_count_);
  this->discarded_non_frame_start_byte_count_ = 0;
}

void MyZoneController::reset_response_parser_() {
  this->response_frame_pos_ = 0;
}

void MyZoneController::reset_command_parser_() {
  this->command_frame_pos_ = 0;
  this->command_frame_started_ms_ = 0;
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
