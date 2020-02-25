////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ExternalClock.h
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_ExternalClock_h
#define Sensors_ExternalClock_h

#include "sensors/SensorSynced.h"
#include <versavis/ExtClk.h>

class ExternalClock : public SensorSynced {
public:
  ExternalClock();
  void setupRos(ros::NodeHandle *nh, const char *topic) override;
  void publish() override;

protected:
  enum class RemoteTimeStatus { kWaiting, kReceived, kTimeout, kReading };
  virtual RemoteTimeStatus setRemoteTime() = 0;
  versavis::ExtClk *clock_msg_ = NULL;

private:
  enum class State {
    kWaitForPulse,
    kWaitForRemoteTime,
    kUpdateFilter,
    kPublishFilterState
  };
  State state_ = State::kWaitForPulse;
};

#endif
