#include "clock_sync/Tcc0Synced.h"
#include "sensors/ExternalClock.h"

ExternalClock::ExternalClock() : SensorSynced(&Tcc0Synced::getInstance()) {
  if (timer_) {
    const bool kInvert = false;
    static_cast<Tcc0Synced *>(timer_)->setupPps(kInvert);
  }
}

void ExternalClock::setupRos(ros::NodeHandle *nh, const char *topic) {
  if (nh) {
    // Create static ROS message.
    static versavis::ExtClk clock_msg;
    clock_msg_ = &clock_msg;

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
