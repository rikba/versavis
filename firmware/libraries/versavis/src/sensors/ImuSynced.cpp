#include "sensors/ImuSynced.h"

ImuSynced::ImuSynced(ros::NodeHandle *nh, TimerSynced *timer)
    : SensorSynced(nh, timer) {}

void ImuSynced::setupRos(const char *topic) {
  if (nh_) {
    // Create static ROS message.
    static sensor_msgs::Imu imu_msg;
    imu_msg_ = &imu_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(topic, imu_msg_);
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(pub);
  }
}
