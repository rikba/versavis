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
    static versavis::ExtClkFilterState filter_state_msg;
    filter_state_msg_ = &filter_state_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(topic, filter_state_msg_);
    publisher_ = &pub;

    // Advertise.
    nh->advertise(pub);
  }
}

void ExternalClock::publish() {
  if (timer_ && filter_state_msg_ &&
      static_cast<TccSynced *>(timer_)->getTimeLastPps(
          &filter_state_msg_->stamp.data, &filter_state_msg_->pps_cnt)) {
    if (publisher_) {
      publisher_->publish(filter_state_msg_);
    }
  }
}
