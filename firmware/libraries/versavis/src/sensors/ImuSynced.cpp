#include "sensors/ImuSynced.h"

ImuSynced::ImuSynced(TimerSynced *timer) : SensorSynced(timer) {}

void ImuSynced::setupRos(ros::NodeHandle *nh, const char *topic) {
  if (nh) {
    // Create static ROS message.
    static sensor_msgs::Imu imu_msg;
    imu_msg_ = &imu_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(topic, imu_msg_);
    publisher_ = &pub;

    // Advertise.
    nh->advertise(pub);
  }
}
