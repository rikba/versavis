////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  RtcSync.h
////////////////////////////////////////////////////////////////////////////////
//
//  This class configures the SAMD21 RTC clock to synchronize against a PPS
//  signal and absolute GNSS time.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef RtcSync_h
#define RtcSync_h

#include <cstdint>

#include <Uart.h>
#include <ros.h>
#include <std_msgs/Time.h>

#include "nmea_parser/NmeaParser.h"

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
  // Setup serial connection to read NMEA sentences.
  void setupNmeaSerial(Uart *uart, const uint32_t baud_rate = 115200);

  // Read UART to synchronize RTC clock against NMEA.
  bool syncGnss();

private:
  RtcSync();

  void setupRos(ros::NodeHandle *nh);
  void setupCounter() const;
  void setupGenericClock5() const;
  void setupRTC() const;

  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher rtc_pub_;
  std_msgs::Time rtc_msg_;

  // NMEA reading
  NmeaParser nmea_parser_;
  bool clear_uart_ = true;
  Uart *uart_ = NULL;
};

#endif
