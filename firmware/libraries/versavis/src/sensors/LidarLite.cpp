#include "sensors/LidarLite.h"

LidarLite::LidarLite(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz)
    : RangeSynced(nh, timer) {}

void LidarLite::publish() {
  if (timer_ && range_msg_ &&
      timer_->getTimeLastTrigger(&range_msg_->header.stamp,
                                 &range_msg_->header.seq)) {
    if (publisher_) {
      publisher_->publish(range_msg_);
    }
  }
}
