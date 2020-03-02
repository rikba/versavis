#include "clock_sync/RtcSync.h"
#include "clock_sync/Tcc0Synced.h"
#include "sensors/ExternalClock.h"

ExternalClock::ExternalClock() : SensorSynced(&Tcc0Synced::getInstance()) {
  if (timer_) {
    const bool kInvert = false;
    static_cast<Tcc0Synced *>(timer_)->setupPps(kInvert);
  }
  resetFilter();
}

void ExternalClock::setupRos(ros::NodeHandle *nh, const char *topic) {
  // Create static ROS message.
  static versavis::ExtClk clock_msg;
  clock_msg_ = &clock_msg;

  if (nh) {

    // Create static ROS publisher.
    static ros::Publisher pub(topic, clock_msg_);
    publisher_ = &pub;

    // Advertise.
    nh->advertise(pub);
  }
}

void ExternalClock::publish() {
  switch (state_) {
  case State::kWaitForPulse: {
    if (timer_ && clock_msg_ &&
        static_cast<TccSynced *>(timer_)->getTimeLastPps(
            &clock_msg_->receive_time, &clock_msg_->pps_cnt)) {
      state_ = State::kWaitForRemoteTime;
    }
    break;
  }
  case State::kWaitForRemoteTime: {
    auto remote_time_status = setRemoteTime();
    if (remote_time_status == RemoteTimeStatus::kReceived) {
      state_ = State::kUpdateFilter;
    } else if (remote_time_status == RemoteTimeStatus::kTimeout) {
      state_ = State::kWaitForPulse;
    }
    break;
  }
  case State::kUpdateFilter: {
    updateFilter();
    controlClock();
    state_ = State::kPublishFilterState;
    break;
  }
  case State::kPublishFilterState: {
    if (publisher_) {
      publisher_->publish(clock_msg_);
    }
    state_ = State::kWaitForPulse;
    break;
  }
  default: {
    state_ = State::kWaitForPulse;
    break;
  }
  }
}

void ExternalClock::updateFilter() {
  if (!clock_msg_)
    return;

  // Sommer, Hannes, et al. "A low-cost system for high-rate, high-accuracy
  // temporal calibration for LIDARs and cameras." 2017 IEEE/RSJ International
  // Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.
  // https://github.com/ethz-asl/cuckoo_time_translator/blob/master/cuckoo_time_translator_algorithms/src/KalmanOwt.cpp#L51-L88

  if (isInitializing()) {
    // Initialize filter.
    auto offset =
        computeDuration(clock_msg_->remote_time, clock_msg_->receive_time);
    measured_offset_s_ = offset.toSec();
    clock_msg_->x[0] = toUSec(offset);
    clock_msg_->x[1] = 0.0;
    clock_msg_->P[0] = pow(RTC_INITIAL_OFFSET * 1.0e6, 2.0);
    clock_msg_->P[1] = 0.0;
    clock_msg_->P[2] = 0.0;
    clock_msg_->P[3] = pow(RTC_MAX_SKEW, 2.0);
  } else {
    // Propagate filter.
    // Prediction.
    clock_msg_->dt = computeDt();

    // x(k) = F(k) * x(k-1)
    clock_msg_->x[0] += clock_msg_->dt * clock_msg_->x[1];

    // P(k) = F(k) * P(k-1) * F(k).transpose + dt * Q
    clock_msg_->P[0] +=
        Q_[0] * clock_msg_->dt + clock_msg_->P[2] * clock_msg_->dt +
        clock_msg_->dt * (clock_msg_->P[1] + clock_msg_->P[3] * clock_msg_->dt);
    clock_msg_->P[1] += clock_msg_->P[3] * clock_msg_->dt;
    clock_msg_->P[2] += clock_msg_->P[3] * clock_msg_->dt;
    clock_msg_->P[3] += Q_[1] * clock_msg_->dt;

    // Measurement update.
    const float S_inv = 1.0 / (R_ + clock_msg_->P[0]);
    float K[2];
    K[0] = clock_msg_->P[0] * S_inv;
    K[1] = clock_msg_->P[2] * S_inv;

    const float residual = toUSec(computeDuration(clock_msg_->remote_time,
                                                  clock_msg_->receive_time)) -
                           clock_msg_->x[0];

    clock_msg_->x[0] += K[0] * residual;
    clock_msg_->x[1] += K[1] * residual;

    clock_msg_->P[2] -= K[1] * clock_msg_->P[0];
    clock_msg_->P[3] -= K[1] * clock_msg_->P[1];
    clock_msg_->P[0] -= K[0] * clock_msg_->P[0];
    clock_msg_->P[1] -= K[0] * clock_msg_->P[1];
  }

  last_update_ = clock_msg_->receive_time;
}

void ExternalClock::controlClock() {
  // Hard reset RTC if more than one second off.
  // TODO(rikba): Replace measurement with offset estimation.
  if (abs(measured_offset_s_) > 1.0) {
    ros::Time reset_time = RtcSync::getInstance().getTimeNow();
    ros::Duration offset;
    offset.fromSec(measured_offset_s_);
    reset_time -= offset;
    RtcSync::getInstance().setTime(reset_time);
    resetFilter();
  } else {
    // TODO(rikba): Implement clock control.
  }
}

void ExternalClock::resetFilter() {
  if (clock_msg_) {
    *clock_msg_ = versavis::ExtClk();
  }
  last_update_ = ros::Time();
}
