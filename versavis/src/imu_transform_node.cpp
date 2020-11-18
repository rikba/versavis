#include "versavis/imu_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  versavis::ImuTransform tf(nh, nh_private);
  ros::spin();
  return 0;
}
