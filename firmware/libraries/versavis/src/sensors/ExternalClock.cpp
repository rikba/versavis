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
  case State::kWaitForPulse:
    if (timer_ && clock_msg_ &&
        static_cast<TccSynced *>(timer_)->getTimeLastPps(
            &clock_msg_->receive_time, &clock_msg_->pps_cnt)) {
      state_ = State::kWaitForRemoteTime;
    }
    break;
  case State::kWaitForRemoteTime:
    if (setRemoteTime() == RemoteTimeStatus::kReceived) {
      state_ = State::kUpdateFilter;
    } else if (setRemoteTime() == RemoteTimeStatus::kTimeout) {
      state_ = State::kWaitForPulse;
    }
    break;
  case State::kUpdateFilter:
    updateFilter();
    controlClock();
    state_ = State::kPublishFilterState;
    break;
  case State::kPublishFilterState:
    if (publisher_) {
      publisher_->publish(clock_msg_);
    }
    state_ = State::kWaitForPulse;
  default:
    state_ = State::kWaitForPulse;
    break;
  }
}

void ExternalClock::updateFilter() {
  if (!clock_msg_)
    return;

  // Sommer, Hannes, et al. "A low-cost system for high-rate, high-accuracy
  // temporal calibration for LIDARs and cameras." 2017 IEEE/RSJ International
  // Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.
  // https://github.com/ethz-asl/cuckoo_time_translator
  // https://github.com/ethz-asl/rosserial/blob/feature/proper-time-sync/rosserial_client/src/ros_lib/ros/node_handle.h#L344-L417

  if (clock_msg_->last_update.sec == 0 && clock_msg_->last_update.nsec == 0) {
    // Initialize filter.
    clock_msg_->x[0] =
        computeDuration(clock_msg_->remote_time, clock_msg_->receive_time)
            .toSec();
    clock_msg_->x[1] = 0.0;
    clock_msg_->P[0] = pow(RTC_INITIAL_OFFSET, 2.0);
    clock_msg_->P[1] = 0.0;
    clock_msg_->P[2] = 0.0;
    clock_msg_->P[3] = pow(RTC_MAX_SKEW, 2.0);
  } else {
    // Propagate filter.
    // Prediction.
    // Measurement.
  }

  // // Prediction.
  // float dt = filter_state_.pps_cnt - filter_state_.pps_cnt_prev;
  // filter_state_.P = filter_state_.P + dt * filter_state_.Q;
  // // Measurement.
  // float y = filter_state_.z - kH * filter_state_.x;
  // float S_inv = 1.0 / (kH * filter_state_.P * kH + filter_state_.R);
  // float K = filter_state_.P * kH * S_inv;
  //
  // filter_state_.x += K * y;
  // filter_state_.P = (1.0 - K * kH) * filter_state_.P;

  clock_msg_->last_update = clock_msg_->receive_time;
}

void ExternalClock::controlClock() {
  // Hard reset RTC if more than one second off.
  // TODO(rikba): Replace measurement with offset estimation.
  if (abs(clock_msg_->x[0]) > 1.0) {
    ros::Time reset_time = RtcSync::getInstance().getTimeNow();
    ros::Duration offset;
    offset.fromSec(clock_msg_->x[0]);
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
}
