////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  CamSynced.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for cameras in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_CamSynced_h
#define Sensors_CamSynced_h

#include "sensors/SensorSynced.h"
#include <versavis/TimeNumbered.h>

class CamSynced : public SensorSynced {
public:
  CamSynced(TimerSynced *timer, const uint16_t rate_hz, const bool invert);
  void setupRos(ros::NodeHandle *nh, const char *topic) override;

protected:
  versavis::TimeNumbered *time_msg_ = NULL;
};

#endif
