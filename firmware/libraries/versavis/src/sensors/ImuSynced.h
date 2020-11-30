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
#include <RingBufCPP.h>
#include <versavis/ImuMicro.h>

class ImuSynced : public SensorSynced {
public:
  ImuSynced(ros::NodeHandle *nh, TimerSynced *timer);
  void setupRos(const char *rate_topic, const char *imu_topic);

protected:
  versavis::ImuMicro *imu_msg_ = NULL;

  RingBufCPP<versavis::ImuMicro, BUFFER_SIZE> imu_buffer_;
};

#endif
