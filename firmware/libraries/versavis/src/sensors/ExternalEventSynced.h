////////////////////////////////////////////////////////////////////////////////
//  July 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ExternalEventSynced.h
////////////////////////////////////////////////////////////////////////////////
//
// Implements an external event interface.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_ExternalEventSynced_h
#define Sensors_ExternalEventSynced_h

#include <ros.h>
#include <std_msgs/Header.h>

#include "clock_sync/TimerSynced.h"

class ExternalEventSynced {
public:
  ExternalEventSynced(ros::NodeHandle *nh, TimerSynced *timer, bool invert);
  void setupRos(char *frame_id, char *event_topic);
  bool publish();

private:
  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher *publisher_ = NULL;

  std_msgs::Header *msg_ = NULL;

  // Timer
  TimerSynced *timer_ = NULL;
};

#endif
