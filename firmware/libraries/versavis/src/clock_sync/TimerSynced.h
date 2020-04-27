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

#include "clock_sync/MeasurementState.h"
#include "clock_sync/MeasurementStateStamped.h"
#include "clock_sync/RtcSync.h"
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

  // Returns true only once per trigger.
  bool getTimeLastTrigger(ros::Time *time, uint32_t *num);

  bool getDataReady(uint32_t *num); // Returns true only once per data ready.

protected:
  void setupWaveOutPin() const;
  bool getPinValue(const uint8_t group, const uint8_t pin) const;
  bool getWaveOutPinValue() const;

  virtual void setupMfrqWaveform() = 0;
  void setupInterruptPin(const uint8_t port_group, const uint8_t pin,
                         const InterruptLogic &logic,
                         const bool enable_interrupt) const;
  virtual void updateTopCompare() = 0;
  void syncRtc();

  // States
  uint8_t prescaler_ = 0;
  uint32_t top_ = 0xFFFF; // Default 16 bit counter.
  uint16_t mod_ = 0;      // The fraction each nominal cycle counts to few.
  uint16_t r_ = 0; // The remainder missing in a cycle to calculate leap ticks.
  uint16_t freq_ = 1; // The rate of the counter.
  // Time since start at overflow.
  ros::Time time_ = RtcSync::getInstance().getTimerStartTime();

  // TODO(rikba): Make these states pointers.
  // Trigger state.
  MeasurementStateStamped trigger_state_;

  // Trigger pin.
  const MfrqPin mfrq_pin_;

  // Data ready state.
  MeasurementState data_ready_;
  uint8_t dr_port_group_ = 0;
  uint8_t dr_pin_ = 0;
};

#endif
