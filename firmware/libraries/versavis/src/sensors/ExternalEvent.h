////////////////////////////////////////////////////////////////////////////////
//  July 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ExternalEvent.h
////////////////////////////////////////////////////////////////////////////////
//
// Implements an external event interface.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_ExternalEvent_h
#define Sensors_ExternalEvent_h

#include <ros.h>
#include <std_msgs/Header.h>

class ExternalEvent {
public:
  ExternalEvent(ros::NodeHandle *nh, TimerSynced *timer, bool invert);
  void setupRos(char *frame_id, char *event_topic);
  bool publish();

private:
  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher *publisher_ = NULL;

  std_msgs::Header *msg_ = NULL;
};

#endif
