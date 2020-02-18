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

  TimerSynced(const MfrqPin &mfrq_pin);

  // Setup the timer.
  virtual void setupDataReady(const uint8_t port_group, const uint8_t pin,
                              const InterruptLogic &logic) = 0;
  void setupMfrq(const uint16_t rate_hz, const bool invert);

  virtual void handleInterrupt() = 0;
  void handleEic();

  inline bool isTriggered() const { return is_triggered_; }
  inline uint32_t getTriggerNumber() const { return trigger_num_; }
  bool hasDataReady();                // resets data_ready_ flag.
  ros::Time computeTimeLastTrigger(); // resets is_triggered_ flag.

protected:
  void syncRtc();
  void overflow();
  void trigger();

  void setupWaveOutPin() const;
  bool getWaveOutPinValue() const;

  virtual void setupMfrqWaveform() const = 0;

  // States
  uint8_t prescaler_ = 0;
  uint32_t top_ = 0xFFFF; // Default 16 bit counter.

  // Trigger state.
  uint16_t rate_hz_ = 0;
  bool invert_trigger_ = false;
  uint32_t trigger_secs_ = 0xFFFFFFFF;
  uint32_t trigger_num_ = 0xFFFFFFFF;
  bool is_triggered_ = false;

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
