#include "sensors/CamSynced.h"

CamSynced::CamSynced(TimerSynced *timer, const uint16_t rate_hz,
                     const bool invert)
    : SensorSynced(timer) {
  // Setup trigger.
  if (timer) {
    timer->setupMfrq(rate_hz, invert);
  }
}

void CamSynced::setupRos(ros::NodeHandle *nh, const char *topic) {
  if (nh) {
    // Create static ROS message.
    static versavis::TimeNumbered time_msg;
    time_msg_ = &time_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(topic, time_msg_);
    publisher_ = &pub;

    // Advertise.
    nh->advertise(pub);
  }
}
