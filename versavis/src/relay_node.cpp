#include <ros/ros.h>

#include "versavis/relay.h"

// Standard C++ entry point
int main(int argc, char **argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "relay_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Creating the relay with ros interface
  versavis::Relay relay(nh, nh_private);
  // Spinning (and processing service calls)
  ros::spin();
  // Exit tranquilly
  return 0;
}
