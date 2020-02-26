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

#include "versavis_configuration.h"

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
  void updateFilter();
  void controlClock();
  void resetFilter();

  enum class State {
    kWaitForPulse,
    kWaitForRemoteTime,
    kUpdateFilter,
    kPublishFilterState
  };
  State state_ = State::kWaitForPulse;

  // Filter tuning.
  // Assume temperature drift of 1 deg / minute.
  const float Q_[2] = {pow(RTC_FREQ_STABILITY * 1.0e-6 / 60.0, 4.0) / 2.0,
                       pow(RTC_FREQ_STABILITY / 60.0, 2.0)}; // [Q11, Q22]
  // Measurement uncertainty is clock resolution + pps accuracy.
  const float R_ = pow(1 / RTC_FREQ + GNSS_PPS_ACCURACY, 2);
};

#endif
