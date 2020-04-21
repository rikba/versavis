////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  SensorSynced.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for generic sensors in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_SensorSynced_h
#define Sensors_SensorSynced_h

#include <ros.h>

#include "clock_sync/TimerSynced.h"

class SensorSynced {
public:
  inline SensorSynced(ros::NodeHandle *nh, TimerSynced *timer)
      : nh_(nh), timer_(timer) {}
  inline virtual void publish() = 0;
  inline virtual void setupRos(const char *topic) = 0;

protected:
  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher *publisher_ = NULL;

  // Timer
  TimerSynced *timer_ = NULL;
};

#endif
