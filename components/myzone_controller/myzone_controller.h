#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace myzone {

class MyZoneController;

// ---------------------------------------------------------------------------
// MyZoneSwitch
// Represents a single zone on/off switch.  State writes are forwarded to the
// controller which handles the actual UART toggle command.
// ---------------------------------------------------------------------------
class MyZoneSwitch : public switch_::Switch {
 public:
  MyZoneSwitch(MyZoneController *controller, uint8_t zone_index)
      : controller_(controller), zone_index_(zone_index) {}

 protected:
  void write_state(bool state) override;

  MyZoneController *controller_;
  uint8_t zone_index_;
};

// ---------------------------------------------------------------------------
// MyZoneController
// Manages UART communication with the MyZone controller and coordinates all
// zone switches.
// ---------------------------------------------------------------------------
class MyZoneController : public Component, public uart::UARTDevice {
 public:
  // ---- ESPHome lifecycle ----
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- Called from __init__.py to register each zone switch ----
  void add_zone_switch(uint8_t zone_index, MyZoneSwitch *sw);

  // ---- Called by MyZoneSwitch::write_state ----
  // Enqueues a toggle command for the given zone if the requested state
  // differs from the currently known state.
  void request_zone_state(uint8_t zone_index, bool requested_state);

 protected:
  // ---- UART helpers ----
  void send_toggle_(uint8_t zone_index);
  void process_byte_(uint8_t byte);
  void process_response_();

  // ---- State helpers ----
  // Parses a 9-byte response and returns the zone bitmask (bits 0-4 = zones
  // 1-5).  Returns 0xFF if the response is ambiguous or unrecognised.
  static uint8_t parse_zone_mask_(const uint8_t *buf);

  // Publish switch states from a zone bitmask.
  void apply_zone_mask_(uint8_t mask);

  // Queue the toggles needed to move from current_mask to target_mask.
  void enqueue_restore_(uint8_t current_mask, uint8_t target_mask);

  // Process the next pending toggle from the queue (if any).
  void dispatch_next_();

  // ---- State machine ----
  enum class State : uint8_t {
    STARTUP_TOGGLE,   // sent zone-1 toggle, awaiting response to learn state
    WAITING_RESPONSE, // sent a user/restore toggle, awaiting response
    IDLE,             // nothing pending
  };

  State state_{State::IDLE};

  // ---- Toggle queue (max 5 pending zones) ----
  // Each entry is a zone index (0-4).
  uint8_t toggle_queue_[5]{};
  uint8_t queue_head_{0};
  uint8_t queue_tail_{0};
  uint8_t queue_size_{0};

  // ---- Currently known zone states (bit i = zone i+1) ----
  uint8_t zone_mask_{0x00};

  // ---- Currently active toggle request ----
  uint8_t active_zone_{0};
  uint32_t request_time_{0};
  uint8_t retry_count_{0};

  // ---- Incoming UART buffer ----
  uint8_t rx_buf_[9]{};
  uint8_t rx_len_{0};

  // ---- Zone switch handles ----
  MyZoneSwitch *zone_switches_[5]{};
  uint8_t num_zones_{0};

  // ---- Persisted state ----
  ESPPreferenceObject zone_mask_pref_;
  uint8_t saved_zone_mask_{0x00};
  bool pref_loaded_{false};

  // ---- Protocol constants ----
  static constexpr uint8_t CMD_PREFIX = 0xC0;
  static constexpr uint8_t RESPONSE_START = 0xFF;
  static constexpr uint8_t RESPONSE_LEN = 9;
  static constexpr uint8_t BUTTON_CODES[5]{0x01, 0x02, 0x04, 0x08, 0x10};
  static constexpr uint8_t MAX_RETRIES = 25;
  static constexpr uint32_t RESPONSE_TIMEOUT_MS = 50;

  // ---- Response lookup table entry ----
  struct ResponseEntry {
    uint8_t response[RESPONSE_LEN];
    uint8_t zone_mask; // 0xFF = ambiguous / unknown
  };

  static const ResponseEntry RESPONSE_TABLE[];
  static const size_t RESPONSE_TABLE_SIZE;
};

}  // namespace myzone
}  // namespace esphome
