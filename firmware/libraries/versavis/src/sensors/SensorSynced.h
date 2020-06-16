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
#include <std_msgs/UInt16.h>

#include "clock_sync/TimerSynced.h"

class SensorSynced {
public:
  inline SensorSynced(ros::NodeHandle *nh, TimerSynced *timer)
      : nh_(nh), timer_(timer) {}
  inline virtual bool publish() = 0;

protected:
  inline void
  setupRos(ros::Subscriber<std_msgs::UInt16, SensorSynced> &rate_sub) {
    if (nh_) {
      timer_->activateLogging(nh_);
      nh_->subscribe(rate_sub);
    }
  }

  inline void changeRateCb(const std_msgs::UInt16 &rate_msg) {
    if (timer_) {
      timer_->updateRate(rate_msg.data);
    }
  }

protected:
  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher *publisher_ = NULL;

  // Timer
  TimerSynced *timer_ = NULL;

private:
};

#endif
