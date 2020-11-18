#include "versavis/mag_transform.h"

#include <ros/ros.h>

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  versavis::MagTransform tf(nh, nh_private);
  ros::spin();
  return 0;
}
