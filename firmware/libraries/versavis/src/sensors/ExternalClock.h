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

#include "clock_sync/MeasurementState.h"
#include "sensors/SensorSynced.h"
#include <versavis/ExtClk.h>

#include "versavis_configuration.h"

class ExternalClock {
public:
  ExternalClock(ros::NodeHandle *nh);
  void setupRos();
  bool publish();

protected:
  enum class RemoteTimeStatus {
    kInitialize,
    kWaiting,
    kReceived,
    kTimeout,
    kReading
  };
  virtual RemoteTimeStatus setRemoteTime() = 0;
  versavis::ExtClk *clock_msg_ = NULL;

  // ROS
  ros::NodeHandle *nh_ = NULL;
  ros::Publisher *publisher_ = NULL;

private:
  // Timer
  TimerSynced *timer_ = NULL;

  void updateFilter();
  void controlClock();
  void resetFilter();

  inline float toUSec(const ros::Duration &duration) const {
    return 1.0e6 * (float)duration.sec + 1.0e-3 * (float)duration.nsec;
  };

  inline float computeDt() const {
    if (clock_msg_) {
      return (clock_msg_->receive_time - last_update_).toSec();
    } else {
      return 0.0;
    }
  }

  enum class State {
    kWaitForPulse,
    kWaitForRemoteTime,
    kUpdateFilter,
    kPublishFilterState
  };
  State state_ = State::kWaitForPulse;

  // Filter tuning.
  // [Q11, Q22, Q33]
  const float Q_[3] = {
      pow(RTC_CLK_SYNC_U_REF / RTC_CLK_SYNC_DAC_RANGE / 2.0, 2.0),
      pow(1.0e-4, 2.0), pow(1.0e-3, 2.0)};
  // Measurement uncertainty is clock resolution + pps accuracy.
  const float R_ =
      pow(1.0e6 / RTC_FREQ / 2.0 + 1.0e6 * GNSS_PPS_ACCURACY / 2.0, 2.0);
  float x_pred_[3];
  float P_pred_[9];
  float z_[1];
  float residual_[1];
  float S_inv_[1];
  float K_[3];

  float measured_offset_s_ = 0.0;
  ros::Time last_update_;
  uint8_t counter_converged_ = 0;

  Measurement measurement_;
};

#endif
