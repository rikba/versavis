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

const uint16_t kPrescalers[8] = {1, 2, 4, 8, 16, 64, 256, 1024};

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
  void setupRos(ros::NodeHandle *nh, const char *topic);
  void publish(); // Resets has_stamp_ flag.

  // Accessors
  void computePwm(const uint16_t rate_hz, const uint32_t pulse_us,
                  const uint16_t prescaler, uint32_t *top,
                  uint32_t *duty_cycle) const;
  ros::Time getTimeNow() const;

  // Setters
  void setSec(const uint32_t sec);
  void start() const;
  void handleEic();

  void incrementMicros();
  void startTimers() const;
  inline ros::Duration getTimeResolution() const { return ros_resolution_; }

  uint8_t findMinPrescalerPwm(const uint16_t rate_hz,
                              const uint32_t counter_max) const;

  uint8_t findMinPrescalerFrq(const uint16_t rate_hz,
                              const uint32_t counter_max) const;

  ros::Duration computeDuration(const uint32_t ticks,
                                const uint8_t prescaler) const;

private:
  RtcSync();

  void setupPort() const;
  void setupGenericClock4() const;
  void setupEvsys() const;
  void setupRtc() const;

  // ROS
  ros::Publisher *rtc_pub_ = NULL;
  std_msgs::Time *rtc_msg_ = NULL;
  bool has_stamp_ = true;

  // State
  ros::Time time_;
  const uint32_t clock_freq_ = RTC_FREQ;
  const uint16_t ns_per_tick_ = 1e9 / clock_freq_;
  const ros::Duration ros_resolution_ = ros::Duration(0, 1000000);
  ros::Time timer_start_ = {1, 0};
  const uint8_t pps_pin_ = 11;
};

#endif
