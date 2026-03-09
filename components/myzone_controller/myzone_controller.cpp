#include "myzone_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace myzone {

static const char *const TAG = "myzone";

// ---------------------------------------------------------------------------
// Response lookup table
//
// Each row maps a 9-byte controller response to the set of zones that are
// currently ACTIVE (bitmask, bit 0 = zone 1 ... bit 4 = zone 5).
//
// Rows whose zone_mask is 0xFF are ambiguous – the same byte sequence appears
// for more than one zone combination in the MyZone firmware.
// ---------------------------------------------------------------------------
const MyZoneController::ResponseEntry MyZoneController::RESPONSE_TABLE[] = {
    // zones 1         FF 08 20 00 00 20 00 D0 F9
    {{0xFF, 0x08, 0x20, 0x00, 0x00, 0x20, 0x00, 0xD0, 0xF9}, 0x01},
    // zones 1+2       FF 08 40 00 00 20 20 D0 F0
    {{0xFF, 0x08, 0x40, 0x00, 0x00, 0x20, 0x20, 0xD0, 0xF0}, 0x03},
    // zones 1+2+3+4   FF 08 40 20 10 20 20 D0 F8
    {{0xFF, 0x08, 0x40, 0x20, 0x10, 0x20, 0x20, 0xD0, 0xF8}, 0x0F},
    // zones 1+2+3+4+5 FF 08 40 40 81 10 08 D0 FA
    {{0xFF, 0x08, 0x40, 0x40, 0x81, 0x10, 0x08, 0xD0, 0xFA}, 0x1F},
    // zones 1+2+3     FF 08 40 00 10 20 00 D0 FA
    {{0xFF, 0x08, 0x40, 0x00, 0x10, 0x20, 0x00, 0xD0, 0xFA}, 0x07},
    // zones 1+2+4     FF 08 40 20 00 28 00 D0 FA
    {{0xFF, 0x08, 0x40, 0x20, 0x00, 0x28, 0x00, 0xD0, 0xFA}, 0x0B},
    // zones 1+3+4     FF 08 40 00 10 28 20 D0 FA
    {{0xFF, 0x08, 0x40, 0x00, 0x10, 0x28, 0x20, 0xD0, 0xFA}, 0x0D},
    // zones 1+4+5     FF 08 40 00 81 10 28 D0 F4
    {{0xFF, 0x08, 0x40, 0x00, 0x81, 0x10, 0x28, 0xD0, 0xF4}, 0x19},
    // zones 1+3       FF 08 40 00 10 20 20 D0 F8
    {{0xFF, 0x08, 0x40, 0x00, 0x10, 0x20, 0x20, 0xD0, 0xF8}, 0x05},
    // zones 1+4       FF 08 40 00 00 28 00 D0 F8
    {{0xFF, 0x08, 0x40, 0x00, 0x00, 0x28, 0x00, 0xD0, 0xF8}, 0x09},
    // zones 1+5       FF 08 40 00 00 20 08 D0 F8
    {{0xFF, 0x08, 0x40, 0x00, 0x00, 0x20, 0x08, 0xD0, 0xF8}, 0x11},
    // zones 2         FF 08 00 00 00 20 20 D0 FA
    {{0xFF, 0x08, 0x00, 0x00, 0x00, 0x20, 0x20, 0xD0, 0xFA}, 0x02},
    // zones 2+4       FF 08 00 20 00 20 00 D0 F8
    {{0xFF, 0x08, 0x00, 0x20, 0x00, 0x20, 0x00, 0xD0, 0xF8}, 0x0A},
    // zones 3         FF 08 00 00 81 00 20 D0 FA
    {{0xFF, 0x08, 0x00, 0x00, 0x81, 0x00, 0x20, 0xD0, 0xFA}, 0x04},
    // zones 3+5       FF 08 00 00 10 20 08 D0 F8
    {{0xFF, 0x08, 0x00, 0x00, 0x10, 0x20, 0x08, 0xD0, 0xF8}, 0x14},
    // zones 4         FF 08 00 00 00 20 00 D0 FA
    {{0xFF, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0xD0, 0xFA}, 0x08},
    // zones 4+5       FF 08 00 00 00 28 08 D0 F8
    {{0xFF, 0x08, 0x00, 0x00, 0x00, 0x28, 0x08, 0xD0, 0xF8}, 0x18},
    // zones 5         FF 08 00 00 00 20 08 D0 F4
    {{0xFF, 0x08, 0x00, 0x00, 0x00, 0x20, 0x08, 0xD0, 0xF4}, 0x10},
    // zones 2+3 and 3+4 share the same response – ambiguous
    // FF 08 00 00 10 20 00 D0 F8
    {{0xFF, 0x08, 0x00, 0x00, 0x10, 0x20, 0x00, 0xD0, 0xF8}, 0xFF},
    // Many multi-zone combinations share this response – ambiguous
    // FF 08 00 01 02 03 04 05 17
    {{0xFF, 0x08, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x17}, 0xFF},
};

const size_t MyZoneController::RESPONSE_TABLE_SIZE =
    sizeof(MyZoneController::RESPONSE_TABLE) / sizeof(MyZoneController::RESPONSE_TABLE[0]);

// ---------------------------------------------------------------------------
// MyZoneSwitch
// ---------------------------------------------------------------------------
void MyZoneSwitch::write_state(bool state) {
  controller_->request_zone_state(zone_index_, state);
}

// ---------------------------------------------------------------------------
// MyZoneController – public API
// ---------------------------------------------------------------------------
void MyZoneController::add_zone_switch(uint8_t zone_index, MyZoneSwitch *sw) {
  if (zone_index < 5) {
    zone_switches_[zone_index] = sw;
    if (zone_index >= num_zones_) {
      num_zones_ = zone_index + 1;
    }
  }
}

void MyZoneController::request_zone_state(uint8_t zone_index, bool requested_state) {
  if (zone_index >= 5) return;

  bool current = (zone_mask_ >> zone_index) & 0x01;
  if (current == requested_state) {
    // Already in the desired state – just confirm it to the switch.
    if (zone_switches_[zone_index] != nullptr) {
      zone_switches_[zone_index]->publish_state(requested_state);
    }
    return;
  }
  // Need to toggle; enqueue it.
  if (queue_size_ < 5) {
    toggle_queue_[queue_tail_] = zone_index;
    queue_tail_ = (queue_tail_ + 1) % 5;
    queue_size_++;
  }
  if (state_ == State::IDLE) {
    dispatch_next_();
  }
}

// ---------------------------------------------------------------------------
// MyZoneController – ESPHome lifecycle
// ---------------------------------------------------------------------------
void MyZoneController::setup() {
  // Load saved zone mask from flash.
  // XOR with a version tag so future schema changes can use a different key.
  zone_mask_pref_ = global_preferences->make_preference<uint8_t>(
      this->get_object_id_hash() ^ 0x4D5A5631UL /* 'MZV1' */);
  if (zone_mask_pref_.load(&saved_zone_mask_)) {
    pref_loaded_ = true;
    ESP_LOGD(TAG, "Loaded saved zone mask: 0x%02X", saved_zone_mask_);
  } else {
    saved_zone_mask_ = 0x00;
    pref_loaded_ = false;
    ESP_LOGD(TAG, "No saved zone mask found");
  }

  // Publish initial states from saved mask so the UI shows the right values
  // even before the controller confirms state.
  for (uint8_t i = 0; i < 5; i++) {
    if (zone_switches_[i] != nullptr) {
      zone_switches_[i]->publish_state((saved_zone_mask_ >> i) & 0x01);
    }
  }
  zone_mask_ = saved_zone_mask_;

  // Send a zone-1 toggle to discover the controller's current state.
  // After receiving the response we know the post-toggle state and can
  // calculate what needs to be restored.
  state_ = State::STARTUP_TOGGLE;
  send_toggle_(0);  // zone 1 = index 0
}

void MyZoneController::loop() {
  // Drain any available UART bytes into the receive buffer.
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    process_byte_(byte);
  }

  // Retry logic: if we are waiting for a response and it hasn't arrived.
  if (state_ == State::WAITING_RESPONSE || state_ == State::STARTUP_TOGGLE) {
    if ((millis() - request_time_) >= RESPONSE_TIMEOUT_MS) {
      if (retry_count_ < MAX_RETRIES) {
        retry_count_++;
        ESP_LOGV(TAG, "Retry %u for zone %u toggle", retry_count_, active_zone_ + 1);
        send_toggle_(active_zone_);
      } else {
        ESP_LOGW(TAG, "No response after %u retries for zone %u toggle", MAX_RETRIES, active_zone_ + 1);
        state_ = State::IDLE;
        retry_count_ = 0;
        // Try to continue with any remaining pending toggles.
        dispatch_next_();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// MyZoneController – protected helpers
// ---------------------------------------------------------------------------
void MyZoneController::send_toggle_(uint8_t zone_index) {
  if (zone_index >= 5) return;
  uint8_t cmd[2] = {CMD_PREFIX, BUTTON_CODES[zone_index]};
  write_array(cmd, 2);
  request_time_ = millis();
  active_zone_ = zone_index;
  ESP_LOGD(TAG, "Sent toggle for zone %u (code 0x%02X)", zone_index + 1, cmd[1]);
}

void MyZoneController::process_byte_(uint8_t byte) {
  if (rx_len_ == 0) {
    if (byte != RESPONSE_START) {
      return;  // Wait for packet start byte.
    }
  }
  rx_buf_[rx_len_++] = byte;
  if (rx_len_ == RESPONSE_LEN) {
    process_response_();
    rx_len_ = 0;
  }
}

void MyZoneController::process_response_() {
  ESP_LOGD(TAG, "Response: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
           rx_buf_[0], rx_buf_[1], rx_buf_[2], rx_buf_[3], rx_buf_[4],
           rx_buf_[5], rx_buf_[6], rx_buf_[7], rx_buf_[8]);

  uint8_t new_mask = parse_zone_mask_(rx_buf_);

  if (state_ == State::STARTUP_TOGGLE) {
    // We sent a zone-1 toggle.  The response reflects the state AFTER that
    // toggle.  Figure out what toggles are needed to reach saved_zone_mask_.
    retry_count_ = 0;
    state_ = State::IDLE;

    if (new_mask != 0xFF) {
      // new_mask is the post-toggle state of the controller.
      // Restore any zone that differs from the desired saved state.
      apply_zone_mask_(new_mask);
      enqueue_restore_(new_mask, saved_zone_mask_);
    } else {
      // Ambiguous response: we can't determine the current state.
      // Fall back to treating the saved state as authoritative and just
      // publish it – we won't attempt UART-level corrections.
      ESP_LOGW(TAG, "Startup: ambiguous response, cannot determine controller state");
      apply_zone_mask_(saved_zone_mask_);
    }
    dispatch_next_();
    return;
  }

  // For WAITING_RESPONSE or an unsolicited response:
  // Treat the response as ground truth and update our state.
  if (new_mask != 0xFF) {
    apply_zone_mask_(new_mask);
  } else {
    ESP_LOGW(TAG, "Ambiguous response received, state not updated");
  }

  if (state_ == State::WAITING_RESPONSE) {
    retry_count_ = 0;
    state_ = State::IDLE;
    dispatch_next_();
  }
}

uint8_t MyZoneController::parse_zone_mask_(const uint8_t *buf) {
  for (size_t i = 0; i < RESPONSE_TABLE_SIZE; i++) {
    bool match = true;
    for (uint8_t j = 0; j < RESPONSE_LEN; j++) {
      if (RESPONSE_TABLE[i].response[j] != buf[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      return RESPONSE_TABLE[i].zone_mask;
    }
  }
  ESP_LOGW(TAG, "Unrecognised response");
  return 0xFF;
}

void MyZoneController::apply_zone_mask_(uint8_t mask) {
  zone_mask_ = mask;
  for (uint8_t i = 0; i < 5; i++) {
    if (zone_switches_[i] != nullptr) {
      zone_switches_[i]->publish_state((mask >> i) & 0x01);
    }
  }
  zone_mask_pref_.save(&zone_mask_);
  ESP_LOGD(TAG, "Zone mask updated to 0x%02X", mask);
}

void MyZoneController::enqueue_restore_(uint8_t current_mask, uint8_t target_mask) {
  uint8_t diff = current_mask ^ target_mask;
  for (uint8_t i = 0; i < 5; i++) {
    if ((diff >> i) & 0x01) {
      if (queue_size_ < 5) {
        toggle_queue_[queue_tail_] = i;
        queue_tail_ = (queue_tail_ + 1) % 5;
        queue_size_++;
      }
      ESP_LOGD(TAG, "Restore: will toggle zone %u", i + 1);
    }
  }
}

void MyZoneController::dispatch_next_() {
  if (queue_size_ == 0) {
    state_ = State::IDLE;
    return;
  }
  uint8_t zone = toggle_queue_[queue_head_];
  queue_head_ = (queue_head_ + 1) % 5;
  queue_size_--;
  state_ = State::WAITING_RESPONSE;
  retry_count_ = 0;
  send_toggle_(zone);
}

// Out-of-class definition required for ODR compliance in C++14.
// C++17+ treats inline constexpr static data members as implicitly inline,
// so this line is harmless there too.
constexpr uint8_t MyZoneController::BUTTON_CODES[5];

}  // namespace myzone
}  // namespace esphome
