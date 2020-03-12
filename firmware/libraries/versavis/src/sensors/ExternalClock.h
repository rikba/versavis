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

#include "clock_sync/MeasurementStateExposure.h"
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

  inline float computeDt() const {
    if (clock_msg_) {
      return computeDuration(last_update_, clock_msg_->receive_time).toSec();
    } else {
      return 0.0;
    }
  }

  inline bool isInitializing() const {
    return last_update_.sec == 0 && last_update_.nsec == 0;
  }

  inline uint16_t computeDacData(const float v_out) const {
    return static_cast<uint16_t>((v_out / 3.3) * 0x3FF) & 0x3FF;
  }

  enum class State {
    kWaitForPulse,
    kWaitForRemoteTime,
    kUpdateFilter,
    kPublishFilterState
  };
  State state_ = State::kWaitForPulse;

  // Filter tuning.
  // Assume temperature drift of 1 deg / minute.
  const float Q_[4] = {pow(RTC_FREQ_STABILITY / 60.0, 4.0) / 2.0,
                       pow(RTC_FREQ_STABILITY / 60.0, 2.0), pow(1.0e-4, 2.0),
                       pow(1.0e-4, 2.0)}; // [Q11, Q22, Q33, Q44]
  // Measurement uncertainty is clock resolution + pps accuracy.
  const float R_ = pow(1.0e6 / RTC_FREQ + 1.0e6 * GNSS_PPS_ACCURACY, 2);
  float x_pred_[4];
  float P_pred_[16];
  float z_[1];
  float residual_[1];
  float S_inv_[1];
  float K_[4];

  float measured_offset_s_ = 0.0;
  ros::Time last_update_;
  uint8_t counter_converged_ = 0;
};

#endif
