#include "versavis/baro_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "baro_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  versavis::BaroTransform tf(nh, nh_private);
  ros::spin();
  return 0;
}
