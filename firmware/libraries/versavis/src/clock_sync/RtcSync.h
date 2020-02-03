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
////////////////////////////////////////////////////////////////////////////////

#ifndef RtcSync_h
#define RtcSync_h

#include <cstdint>

#include <Uart.h>
#include <ros.h>
#include <std_msgs/Time.h>

#include "nmea_parser/NmeaParser.h"
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

  // Setup the ROS publishers, and RTC counter.
  void setup(ros::NodeHandle *nh);
  inline uint32_t getSecs() const { return secs_; }
  inline uint32_t setSecs(const uint32_t secs) { secs_ = secs; }
  inline uint32_t getComp0() const { return RTC->MODE0.COMP[0].reg; }
  void setComp0(const uint32_t comp_0) const;
  inline void incrementSecs() { secs_++; }

private:
  RtcSync();

  void setupRos(ros::NodeHandle *nh);
  void setupPort() const;
  void setupGenericClock5() const;
  void setupEvsys() const;
  void setupRtc() const;

  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher rtc_pub_;
  std_msgs::Time rtc_msg_;

  // State
  uint32_t secs_ = 0;
  double ns_per_tick_ = 10e9 / RTC_FREQ;
};

#endif
