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

  virtual void handleInterrupt() = 0;

  // Synchronize timer against RTC clock. To be called in main loop.
  void sync();

private:
  // State
  ros::Time stamp_ = ros::Time(0, 0); // Absolute time at cycle start.
  bool sync_rtc_stamp_ = false;
};

#endif
