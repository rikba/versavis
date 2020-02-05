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

class TimerSynced {
public:
  TimerSynced();

  // Setup the timer.
  virtual void setup() const = 0;
  void handleInterrupt();

  inline bool isTriggered() const { return is_triggered_; }
  ros::Time computeTimeLastTrigger(); // resets is_triggered_ flag.

protected:
  virtual void handleRetrigger() = 0;
  virtual void handleOverflow() = 0;
  void retrigger();
  void overflow();

  const uint16_t kPrescalers[8] = {1, 2, 4, 8, 16, 64, 256, 1024};
  uint8_t prescaler_ = 0;
  uint32_t top_ = 0xFFFF; // Default 16 bit counter.
  // Flag to store whether trigger stamp has been requested.
  bool is_triggered_ = false;

private:
  // State
  uint32_t ovf_ticks_since_sync_ = 0;
};

#endif
