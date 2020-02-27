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

  inline float toUSec(const ros::Duration &duration) const {
    return 1.0e6 * (float)duration.sec + 1.0e-3 * (float)duration.nsec;
  };

  inline ros::Duration fromUSec(float t) {
    uint32_t sec = 1.0e-6 * (uint32_t)floor(t);
    uint32_t nsec = (uint32_t)round((1.0e-6 * t - sec) * 1e9);
    ros::Duration duration(sec, nsec);
  };

  enum class State {
    kWaitForPulse,
    kWaitForRemoteTime,
    kUpdateFilter,
    kPublishFilterState
  };
  State state_ = State::kWaitForPulse;

  // Filter tuning.
  // Assume temperature drift of 1 deg / minute.
  const float Q_[2] = {pow(RTC_FREQ_STABILITY / 60.0, 4.0) / 2.0,
                       pow(RTC_FREQ_STABILITY / 60.0, 2.0)}; // [Q11, Q22]
  // Measurement uncertainty is clock resolution + pps accuracy.
  const float R_ = pow(1.0e6 / RTC_FREQ + 1.0e6 * GNSS_PPS_ACCURACY, 2);

  float measured_offset_s_ = 0.0;
  ros::Time last_update_;
};

#endif
