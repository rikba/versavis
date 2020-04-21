////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ImuSynced.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for IMUs in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_ImuSynced_h
#define Sensors_ImuSynced_h

#include "sensors/SensorSynced.h"
#include <sensor_msgs/Imu.h>

class ImuSynced : public SensorSynced {
public:
  ImuSynced(ros::NodeHandle *nh, TimerSynced *timer);
  void setupRos(const char *topic) override;

protected:
  sensor_msgs::Imu *imu_msg_ = NULL;
};

#endif
