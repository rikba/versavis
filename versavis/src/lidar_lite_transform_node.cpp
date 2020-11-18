#include "versavis/lidar_lite_transform.h"

#include <ros/ros.h>

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_lite_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  versavis::LidarLiteTransform tf(nh, nh_private);
  ros::spin();
  return 0;
}
