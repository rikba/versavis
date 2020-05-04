#include "sensors/RangeSynced.h"

RangeSynced::RangeSynced(ros::NodeHandle *nh, TimerSynced *timer)
    : SensorSynced(nh, timer) {}

void RangeSynced::setupRos(const char *topic) {
  if (nh_) {
    // Create static ROS message.
    static sensor_msgs::Range range_msg;
    range_msg_ = &range_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(topic, range_msg_);
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(pub);
  }
}
