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

const uint16_t kPrescalers[8] = {1, 2, 4, 8, 16, 64, 256, 1024};

class TimerSynced {
public:
  TimerSynced();

  // Setup the timer.
  virtual void setup() const = 0;
  virtual void setupPwm(uint16_t rate_hz, uint32_t pulse_us, bool invert) = 0;

  virtual void handleInterrupt() = 0;

  inline bool isTriggered() const { return is_triggered_; }
  ros::Time computeTimeLastTrigger(); // resets is_triggered_ flag.

  static uint8_t findMinPrescaler(const uint16_t rate_hz,
                                  const uint32_t clock_freq,
                                  const uint32_t counter_max);

protected:
  void syncRtc();
  void overflow();
  void pwmPulse();

  uint8_t prescaler_ = 0;
  uint32_t top_ = 0xFFFF; // Default 16 bit counter.
  // Flag to store whether trigger stamp has been requested.
  bool is_triggered_ = false;

private:
  // State
  uint32_t ovf_ticks_since_sync_ = 0;
  uint32_t rtc_secs_ = 0xFFFFFFFF;
};

#endif
