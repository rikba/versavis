#ifndef VERSAVIS_RELAY_H_
#define VERSAVIS_RELAY_H_

#include <memory>
#include <queue>

#include <camera_info_manager/camera_info_manager.h>
#include <image_numbered_msgs/ImageNumbered.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

namespace versavis {

class Relay {
public:
  Relay(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  void initialize();

private:
  enum class State {
    kInitWaitFirstMsg = 0,
    kInitSlowRate,
    kInitWaitSlowRate,
    kInitClear,
    kInitWaitForCorrespondance,
    kRunning
  };

  void loadParameters();
  void subscribeToTopics();
  void advertiseTopics();

  void imageHeaderCb(const std_msgs::Header::ConstPtr &msg);
  void imageCb(const image_numbered_msgs::ImageNumbered::ConstPtr &msg);

  void setRate(const int rate);
  void associate();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber img_header_sub_;
  ros::Subscriber img_sub_;

  ros::Publisher img_rate_pub_;
  ros::Publisher img_seq_pub_;

  image_transport::ImageTransport it_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  image_transport::CameraPublisher img_pub_;
  image_transport::CameraPublisher img_throttle_pub_;

  std::deque<std_msgs::Header::ConstPtr> headers_;
  std::deque<image_numbered_msgs::ImageNumbered::ConstPtr> images_;

  State state_ = State::kInitWaitFirstMsg;

  // Parameters.
  int rate_ = 10;

  // Throttle counter
  int throttle_ = -1;
  int throttle_cnt_ = 0;
};

} // namespace versavis

#endif // VERSAVIS_RELAY_H_
