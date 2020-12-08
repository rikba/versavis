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
  inline void setMeasurementState(const MeasurementState &state) {
    measurement_state_ = state;
  }
  virtual void setupDataReady(const uint8_t port_group, const uint8_t pin,
                              const InterruptLogic &logic) = 0;
  void setupMfrq(const uint16_t rate_hz);
  void setupMpwm(const uint16_t rate_hz, const uint16_t pulse_us);
  void setLatestMeasurementNum(const uint32_t num);
  void offsetTrigger(const double sec);

  virtual void handleInterrupt() = 0;
  void handleEic();

  // Receive oldest measurement. Only returns valid measurements. Clears this
  // and older measurements from buffer.
  bool getMeasurement(Measurement *meas);

  // Change rate to the closest possible rate.
  virtual void updateRate(const uint16_t rate_hz) = 0;

  inline void activateLogging(ros::NodeHandle *nh) { nh_ = nh; }
  bool getPinValue(const uint8_t group, const uint8_t pin) const;

protected:
  // Does not change prescaler. Sets rate to the closest possible rate.
  void setClosestRateMfreq(const uint16_t rate_hz);
  void setClosestRateMpwm(const uint16_t rate_hz);
  bool updateFreq();
  void updateRateMfrq(const uint16_t rate_hz);
  void updateRateMpwm(const uint16_t rate_hz);
  void setupWaveOutPin() const;
  bool getWaveOutPinValue() const;
  uint16_t computeLeapTicks();

  virtual void setupMfrqWaveform() = 0;
  virtual void setupMpwmWaveform() = 0;
  void setupInterruptPin(const uint8_t port_group, const uint8_t pin,
                         const InterruptLogic &logic,
                         const bool enable_interrupt) const;
  virtual void updateTopCompare() = 0;
  void syncRtc();

  // States
  uint8_t prescaler_ = 0;
  uint32_t top_max_ = 0xFFFF; // Default 16 bit counter.
  uint32_t top_ = top_max_;
  uint16_t mod_ = 0; // The fraction each nominal cycle counts to little.
  uint32_t wrap_around_ = 0; // The wrap around when a leap tick should occur.
  uint16_t r_ = 0; // The remainder missing in a cycle to calculate leap ticks.
  uint16_t freq_ = 0; // The rate of the counter.
  uint16_t new_freq_ = 0;
  uint16_t pulse_ticks_ = 0; // Pulse length in PWM mode.
  ros::Time time_ = {1, 0};  // Timers are started by RTC at first second.

  // The default measurement state handling a single sensor input.
  MeasurementState measurement_state_;

  // Trigger pin.
  const MfrqPin mfrq_pin_;
  int32_t offset_ = 0;
  int32_t accumulated_offset_ = 0;

  // Data ready state.
  uint8_t dr_port_group_ = 0;
  uint8_t dr_pin_ = 0;

  // Logging.
  ros::NodeHandle *nh_ = NULL;

private:
  void setClosestRate(const uint16_t freq);
};

#endif
