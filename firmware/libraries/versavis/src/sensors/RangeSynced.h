////////////////////////////////////////////////////////////////////////////////
//  April 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  RangeSynced.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for range sensors in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_RangeSynced_h
#define Sensors_RangeSynced_h

#include "sensors/SensorSynced.h"
#include <sensor_msgs/Range.h>

class RangeSynced : public SensorSynced {
public:
  RangeSynced(ros::NodeHandle *nh, TimerSynced *timer);
  void setupRos(const char *topic) override;

protected:
  sensor_msgs::Range *range_msg_ = NULL;
};

#endif
