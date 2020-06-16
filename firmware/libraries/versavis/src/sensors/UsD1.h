////////////////////////////////////////////////////////////////////////////////
//  May 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  UsD1.h
////////////////////////////////////////////////////////////////////////////////
//
// Implements the UARD US-D1 radar interface. Currently, we poll the UART at
// maximum rate (82 Hz).
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_UsD1_h
#define Sensors_UsD1_h

#include <versavis/UsD1.h>
#include <ros.h>

enum class UsD1State { kHeader, kVersion, kLsb, kMsb, kSnr, kCs };

class UsD1 {
public:
  UsD1(ros::NodeHandle *nh, Uart *uart);
  void setupRos(const char *topic);
  bool publish();

private:
  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher *publisher_ = NULL;

  versavis::UsD1 *msg_ = NULL;
  Uart *uart_ = NULL;

  uint8_t lsb_ = 0xFF;
  uint16_t cs_ = 0;
  UsD1State state_ = UsD1State::kHeader;
};

#endif
