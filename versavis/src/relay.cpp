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

  nh_private_.getParam("throttle", throttle_);
  ROS_INFO("Camera throttle: %d", throttle_);

  // Camera info.
  cinfo_.reset(
      new camera_info_manager::CameraInfoManager(nh_private_.getNamespace()));
  std::string camera_name = "cam0";
  if (!nh_private_.getParam("camera_name", camera_name)) {
    ROS_WARN_STREAM(
        "Using default camera name in camera info: " << camera_name);
  }
  cinfo_->setCameraName(camera_name);

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
  img_pub_ = it_.advertiseCamera("image_synced", kBufferSize, kLatchTopic);
  ROS_INFO("Relaying images with corrected stamp to: %s",
           img_pub_.getTopic().c_str());

  img_throttle_pub_ =
      it_.advertiseCamera("image_synced_throttle", kBufferSize, kLatchTopic);
  ROS_INFO("Relaying throttled images with corrected stamp to: %s",
           img_throttle_pub_.getTopic().c_str());

  img_rate_pub_ =
      nh_.advertise<std_msgs::UInt16>("set_rate", kBufferSize, kLatchTopic);

  img_seq_pub_ =
      nh_.advertise<std_msgs::UInt32>("set_seq", kBufferSize, kLatchTopic);
}

void Relay::setRate(const int rate) {
  ROS_INFO("Setting camera rate to %dHz: %s", rate,
           img_rate_pub_.getTopic().c_str());
  std_msgs::UInt16 new_rate;
  new_rate.data = rate;
  img_rate_pub_.publish(new_rate);
}

void Relay::imageHeaderCb(const std_msgs::Header::ConstPtr &msg) {
  ROS_INFO_ONCE("Received first image header from micro controller.");
  headers_.push_back(msg);
  if (state_ == State::kRunning) {
    associate();
  }
  if (headers_.size() > kBufferSize) {
    ROS_WARN("Header deque overflowing. Removing oldest.");
    headers_.pop_front();
  }
}

void Relay::imageCb(const image_numbered_msgs::ImageNumbered::ConstPtr &msg) {
  ROS_INFO_ONCE("Received first image from camera.");
  images_.push_back(msg);
  if (state_ == State::kRunning) {
    associate();
  }
  if (images_.size() > kBufferSize) {
    ROS_WARN("Image deque overflowing. Removing oldest.");
    images_.pop_front();
  }
}

void Relay::associate() {
  for (auto img_it = images_.begin(); img_it != images_.end(); img_it++) {
    // Find matching image numbers.
    auto header_it =
        std::find_if(headers_.begin(), headers_.end(), [&](const auto &header) {
          return header->seq == (*img_it)->number;
        });

    // On match, publish restamped image and erase matched entries.
    if (header_it != headers_.end()) {
      // Create copy of image with new stamp.
      sensor_msgs::Image img((*img_it)->image);
      img.header = **header_it;
      sensor_msgs::CameraInfo ci(cinfo_->getCameraInfo());
      ci.header = img.header;
      img_pub_.publish(img, ci);
      if (throttle_ > 0 && (throttle_cnt_++ % throttle_)) {
        img_throttle_pub_.publish(img, ci);
      }
      // Erase all images up to and including the current image.
      img_it = images_.erase(images_.begin(), img_it + 1);
      // Erase all headers up to and including the current stamp.
      headers_.erase(headers_.begin(), header_it + 1);
    }
    if (img_it == images_.end())
      break;
  }
}

// This is (a little bit brittle) initialization procedure.
// 1. Reduce camera rate
// 2. Find images and headers with close time stamps.
// 3. Set microcontroller image number to current image number.
// 4. Reset rate and exit.
void Relay::initialize() {
  ros::Rate loop_rate(1000);
  ros::Time start;

  while (ros::ok() && state_ != State::kRunning) {
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
        corresponding &= std::fabs((headers_.back()->stamp -
                                    images_.back()->image.header.stamp)
                                       .toSec()) < kMaxImageDelayThreshold;
      }

      if (corresponding) {
        ROS_INFO("Found corresponding image %lu and header %u.",
                 images_.back()->number, headers_.back()->seq);

        ROS_INFO("Setting image number to %lu: %s", images_.back()->number,
                 img_seq_pub_.getTopic().c_str());
        std_msgs::UInt32 new_seq;
        new_seq.data = images_.back()->number;
        img_seq_pub_.publish(new_seq);

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
