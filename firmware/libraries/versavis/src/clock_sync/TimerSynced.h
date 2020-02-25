////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TimerSynced.h
////////////////////////////////////////////////////////////////////////////////
//
//  This class configures a timer that is synchronized against RTC clock and can
//  be used to trigger and stamp sensor data.
//
//  - Retrigger on RTC OvF event
//  - Store time stamp of current cycle start
//  - Read absolute time from RTC
//  - Match PWM for trigger generation, interrupt event capture for event in
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TimerSynced_h
#define TimerSynced_h

#include <cstdint>

#include <ros.h>

#include "clock_sync/Timestamp.h"
#include "versavis_configuration.h"

enum class InterruptLogic {
  kNone = 0x0u,
  kRise = 0x1u,
  kFall = 0x2u,
  kBoth = 0x3u,
  kHigh = 0x4u,
  kLow = 0x5u
};

class TimerSynced {
public:
  // TODO(rikba): Add default values in C++14.
  struct MfrqPin {
    uint8_t group;
    uint8_t pin;
    bool drvstr;
  };

  class TriggerState {
  public:
    inline void syncRtc() { trigger_.syncRtc(); }
    inline void overflow() { trigger_.overflow(); }
    inline void trigger(const uint8_t prescaler, const uint32_t top) {
      // Compute time stamp based on trigger number.
      const uint16_t trigger_in_second = trigger_counter_ % rate_hz_;
      const uint32_t cc = (trigger_in_second * 2 + 1) * (top + 1);
      trigger_.setTicks(cc);
      is_triggered_ = trigger_.computeTime(prescaler, top, &trigger_time_);
      trigger_counter_++;
    }

    inline bool getTime(ros::Time *time, uint32_t *trigger_num) {
      if (is_triggered_ && time) {
        *time = trigger_time_;
      }
      if (is_triggered_ && trigger_num) {
        *trigger_num = trigger_counter_ - 1; // Start counting from 0.
      }

      bool success = is_triggered_;
      is_triggered_ = false;
      return success;
    }

    bool invert_ = false;
    uint16_t rate_hz_ = 0;

  private:
    Timestamp trigger_;
    ros::Time trigger_time_;
    bool is_triggered_ = false;

    uint32_t trigger_counter_ = 0xFFFFFFFF;
  };

  TimerSynced(const MfrqPin &mfrq_pin);

  // Setup the timer.
  virtual void setupDataReady(const uint8_t port_group, const uint8_t pin,
                              const InterruptLogic &logic) = 0;
  void setupMfrq(const uint16_t rate_hz, const bool invert);

  virtual void handleInterrupt() = 0;
  void handleEic();

  // Returns true only once per trigger.
  inline bool computeTimeLastTrigger(ros::Time *time, uint32_t *trigger_num) {
    return trigger_state_.getTime(time, trigger_num);
  }

  bool hasDataReady(); // resets data_ready_ flag.

protected:
  void setupWaveOutPin() const;
  bool getPinValue(const uint8_t group, const uint8_t pin) const;
  bool getWaveOutPinValue() const;

  virtual void setupMfrqWaveform() const = 0;
  void setupInterruptPin(const uint8_t port_group, const uint8_t pin,
                         const InterruptLogic &logic,
                         const bool enable_interrupt) const;

  // States
  uint8_t prescaler_ = 0;
  uint32_t top_ = 0xFFFF; // Default 16 bit counter.

  // Trigger state.
  TriggerState trigger_state_;

  // Trigger pin.
  const MfrqPin mfrq_pin_;

  // Data ready state.
  bool data_ready_ = false;
  uint8_t dr_port_group_ = 0;
  uint8_t dr_pin_ = 0;

  // Free running state.
  uint32_t ovf_ticks_since_sync_ = 0;
};

#endif
