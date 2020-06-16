#include "sensors/CamSynced.h"

CamSynced::CamSynced(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz, const bool invert)
    : SensorSynced(nh, timer) {
  // Setup trigger.
  if (timer) {
    timer->setupMfrq(rate_hz, invert);
  }
}

void CamSynced::setupRos(char *rate_topic, char *img_topic) {
  static ros::Subscriber<std_msgs::UInt16, SensorSynced> rate_sub(
      rate_topic, &CamSynced::changeRateCb, this);
  SensorSynced::setupRos(rate_sub);
  if (nh_) {
    // Create static ROS message.
    static versavis::TimeNumbered time_msg;
    time_msg_ = &time_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(img_topic, time_msg_);
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(pub);
  }
}
