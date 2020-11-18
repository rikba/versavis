#include "versavis/us_d1_transform.h"

#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "us_d1_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  versavis::UsD1Transform tf(nh, nh_private);
  ros::spin();
  return 0;
}
