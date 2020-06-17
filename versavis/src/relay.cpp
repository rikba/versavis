#include "versavis/relay.h"

#include <cmath>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>

const size_t kBufferSize = 100;
const bool kLatchTopic = true;
const double kMaxImageDelayThreshold = 0.1;

namespace versavis {

Relay::Relay(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), it_(nh_) {
  loadParameters();
  advertiseTopics();
  subscribeToTopics();
}

void Relay::loadParameters() {
  // Parameters
  nh_private_.getParam("rate", rate_);
  ROS_INFO("Camera rate: %d", rate_);

  // Camera info.
  cinfo_.reset(
      new camera_info_manager::CameraInfoManager(nh_private_.getNamespace()));
  std::string cam_name = "cam0";
  if (!nh_private_.getParam("camera_name", cam_name)) {
    ROS_WARN_STREAM("Using default camera name in camera info: " << cam_name);
  }
  cinfo_->setCameraName(cam_name);

  std::string cam_info_url;
  if (nh_private_.getParam("camera_info_url", cam_info_url)) {
    if (cinfo_->validateURL(cam_info_url)) {
      cinfo_->loadCameraInfo(cam_info_url);
    } else {
      ROS_WARN_STREAM("Invalid camera_info_url.");
    }
  }
}

void Relay::subscribeToTopics() {
  img_sub_ =
      nh_.subscribe("image_numbered", kBufferSize, &Relay::imageCb, this);
  ROS_INFO("Subscribing to image topic: %s", img_sub_.getTopic().c_str());
  img_header_sub_ =
      nh_.subscribe("header", kBufferSize, &Relay::imageHeaderCb, this);
  ROS_INFO("Subscribing to image header topic: %s",
           img_header_sub_.getTopic().c_str());
}

void Relay::advertiseTopics() {
  img_pub_ = it_.advertiseCamera("cam0/image_raw", kBufferSize, kLatchTopic);
  ROS_INFO("Relaying images with corrected stamp to: %s",
           img_pub_.getTopic().c_str());
}

void Relay::setRate(const int rate) {
  ros::Publisher img_rate_pub =
      nh_.advertise<std_msgs::UInt16>("set_rate", kBufferSize, kLatchTopic);

  ROS_INFO("Setting camera rate to %dHz: %s", rate,
           img_rate_pub.getTopic().c_str());
  std_msgs::UInt16 new_rate;
  new_rate.data = rate;
  img_rate_pub.publish(new_rate);
}

void Relay::imageHeaderCb(const std_msgs::Header &msg) {
  ROS_INFO_ONCE("Received first image header from micro controller.");
  headers_.push_back(msg);
  if (state_ == State::kRunning) {
    associate();
  }
}

void Relay::imageCb(const image_numbered_msgs::ImageNumbered &msg) {
  ROS_INFO_ONCE("Received first image from camera.");
  images_.push_back(msg);
  if (state_ == State::kRunning) {
    associate();
  }
}

void Relay::associate() {
  for (auto img_it = images_.begin(); img_it != images_.end(); img_it++) {
    // Find matching image numbers.
    auto header_it =
        std::find_if(headers_.begin(), headers_.end(), [&](const auto &header) {
          return header.seq == img_it->number;
        });

    // On match, publish restamped image and erase matched entries.
    if (header_it != headers_.end()) {
      img_it->image.header = *header_it;
      sensor_msgs::CameraInfo ci(cinfo_->getCameraInfo());
      ci.header = img_it->image.header;
      img_pub_.publish(img_it->image, ci);
      // Erase all images up to and including the current image.
      img_it = images_.erase(images_.begin(), img_it);
      // Erase all headers up to and including the current stamp.
      headers_.erase(headers_.begin(), header_it);
    }
  }

  ROS_ERROR_COND(images_.size() > kBufferSize, "Image deque not cleared.");
  ROS_ERROR_COND(headers_.size() > kBufferSize, "Header deque not cleared.");
}

// This is (a little bit brittle) initialization procedure.
// 1. Reduce camera rate
// 2. Find images and headers with close time stamps.
// 3. Set microcontroller image number to current image number.
// 4. Reset rate and exit.
void Relay::initialize() {
  ros::Rate loop_rate(1000);
  ros::Time start;

  while (state_ != State::kRunning) {
    switch (state_) {
    case State::kInitWaitFirstMsg: {
      if (!headers_.empty() && !images_.empty()) {
        state_ = State::kInitSlowRate;
      } else {
        ROS_WARN_THROTTLE(1.0, "Waiting to receive first messages.");
      }
      break;
    }
    case State::kInitSlowRate: {
      setRate(1);
      start = ros::Time::now();
      state_ = State::kInitWaitSlowRate;
      break;
    }
    case State::kInitWaitSlowRate: {
      ROS_INFO_ONCE("Waiting for rate change to take effect.");
      if ((ros::Time::now() - start).toSec() > 2.0) {
        headers_.clear();
        images_.clear();
        state_ = State::kInitWaitForCorrespondance;
      }
      break;
    }
    case State::kInitWaitForCorrespondance: {
      // Wait until an image-header-pair arrives that has close time stamps.
      bool corresponding = !headers_.empty() && !images_.empty();
      if (corresponding) {
        corresponding &= std::fabs((headers_.back().stamp -
                                    images_.back().image.header.stamp)
                                       .toSec()) < kMaxImageDelayThreshold;
      }

      if (corresponding) {
        ROS_INFO("Found corresponding image %lu and header %u.",
                 images_.back().number, headers_.back().seq);
        ros::Publisher img_seq_pub = nh_.advertise<std_msgs::UInt32>(
            "set_seq", kBufferSize, kLatchTopic);

        ROS_INFO("Setting image number to %lu: %s", images_.back().number,
                 img_seq_pub.getTopic().c_str());
        std_msgs::UInt32 new_seq;
        new_seq.data = images_.back().number;
        img_seq_pub.publish(new_seq);

        headers_.clear();
        images_.clear();

        setRate(rate_);
        state_ = State::kRunning;
      } else {
        ROS_INFO_THROTTLE(1.0,
                          "Waiting for first image header correspondance.");
      }
      break;
    }
    default: {
      ROS_FATAL("Unhandled state %d", static_cast<int>(state_));
      ros::shutdown();
      break;
    }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

} // namespace versavis
