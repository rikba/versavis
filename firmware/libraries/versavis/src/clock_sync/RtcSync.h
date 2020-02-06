////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  RtcSync.h
////////////////////////////////////////////////////////////////////////////////
//
//  This class configures the SAMD21 RTC to deliver absolute time information to
//  VersaVIS peripherals.
//
//  - COMP0 match clear register to reset at 1 Hz
//  - Count total seconds
//  - Output event to other timers on COMP0
//  - Interface to control compare register and total seconds
//
////////////////////////////////////////////////////////////////////////////////

#ifndef RtcSync_h
#define RtcSync_h

#include <cstdint>

#include <Uart.h>
#include <ros.h>
#include <ros/duration.h>
#include <std_msgs/Time.h>

#include "versavis_configuration.h"

class RtcSync {
public:
  // Singleton implementation.
  inline static RtcSync &getInstance() {
    static RtcSync instance;
    return instance;
  }
  RtcSync(RtcSync const &) = delete;
  void operator=(RtcSync const &) = delete;

  // ROS
  void setupRos(ros::NodeHandle *nh);
  void publish();

  // Accessors
  inline uint32_t getSecs() const { return secs_; }
  inline uint32_t getComp0() const { return RTC->MODE0.COMP[0].reg; }
  ros::Time computeTime(const uint32_t ticks, uint16_t prescaler) const;
  void computePwm(const uint16_t rate_hz, const uint32_t pulse_us,
                  const uint16_t prescaler, uint32_t *top,
                  uint32_t *duty_cycle) const;
  inline ros::Time getTimeSecs() const { return ros::Time(secs_, 0); }

  // Setters
  inline uint32_t setSecs(const uint32_t secs) { secs_ = secs; }
  void setComp0(const uint32_t comp_0) const;

  inline void incrementSecs() { secs_++; }

private:
  RtcSync();

  void setupPort() const;
  void setupGenericClock4() const;
  void setupEvsys() const;
  void setupRtc() const;

  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher rtc_pub_;
  std_msgs::Time rtc_msg_;

  // State
  uint32_t secs_ = 0;
  uint32_t clock_freq_ = RTC_FREQ;
  double ns_per_tick_ = 1.0e9 / clock_freq_;
};

#endif
