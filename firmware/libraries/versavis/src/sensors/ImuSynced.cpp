#include "sensors/ImuSynced.h"

ImuSynced::ImuSynced(TimerSynced *timer) : SensorSynced(timer) {
  static versavis::ImuMicro imu_msg;
  imu_msg_ = &imu_msg;
}

void ImuSynced::setupRos(ros::NodeHandle *nh, char *topic) {
  if (nh) {
    static char *static_topic = topic;
    static ros::Publisher pub(static_topic, imu_msg_);
    publisher_ = &pub;
    nh->advertise(pub);
  }
}
