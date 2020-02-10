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
#include <versavis/ImuMicro.h>

class ImuSynced : public SensorSynced {
public:
  ImuSynced(TimerSynced *timer);
  void setupRos(ros::NodeHandle *nh, const char *topic) override;

protected:
  versavis::ImuMicro *imu_msg_ = NULL;
};

#endif
