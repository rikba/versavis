#include "versavis/temp_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "temp_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  versavis::TempTransform tf(nh, nh_private);
  ros::spin();
  return 0;
}
