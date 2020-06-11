#include "sensors/UsD1.h"

#include "clock_sync/RtcSync.h"

const uint8_t kHeader = 0xFE;
const uint8_t kVersionId = 0x02;
const float kVarianceLow = pow(0.06 / 3, 2);
const float kVarianceHigh = pow(0.04 / 3, 2);

UsD1::UsD1(ros::NodeHandle *nh, Uart *uart) : nh_(nh), uart_(uart) {
  if (uart_) {
    uart_->begin(115200);
  }
}

void UsD1::setupRos(const char *topic) {
  if (nh_) {
    // Create static ROS msg.
    static versavis::UsD1 msg;

    // Assign topic pointer.
    msg_ = &msg;

    // Create static ROS publisher.
    static ros::Publisher pub(topic, msg_);

    // Assign publisher pointers.
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(*publisher_);
  }

  // Initialize.
  if (msg_) {
    msg_->range.header.frame_id = "US-D1";

    // https://cdn.shopify.com/s/files/1/0113/0414/0900/files/User_Manual_US-D1.pdf?16288212927919010227
    // TODO(rikba): Fix sensor type https://github.com/ros/common_msgs/pull/153
    msg_->range.radiation_type = sensor_msgs::Range::INFRARED;
    msg_->range.field_of_view = 0.35;
    msg_->range.min_range = 0.5;
    msg_->range.max_range = 50.0;
  }
}

void UsD1::publish() {

  while (msg_ && uart_ && uart_->available()) {
    uint8_t c = uart_->read();

    // Parse character.
    switch (state_) {
    case UsD1State::kHeader: {
      if (c == kHeader) {
        msg_->range.header.stamp = RtcSync::getInstance().getTimeNow();
        cs_ = 0; // Reset checksum.
        state_ = UsD1State::kVersion;
      }
      break;
    }
    case UsD1State::kVersion: {
      if (c == kVersionId) {
        cs_ += c;
        state_ = UsD1State::kLsb;
      } else {
        if (nh_) {
          char buffer[100];
          sprintf(buffer,
                  "Failed to read US-D1 version ID. Expected: %d Is: %d",
                  kVersionId, c);
          nh_->logwarn(buffer);
        }
        state_ = UsD1State::kHeader;
      }
      break;
    }
    case UsD1State::kLsb: {
      lsb_ = c;
      cs_ += c;
      state_ = UsD1State::kMsb;
      break;
    }
    case UsD1State::kMsb: {
      msg_->range.range = ((c << 8) + lsb_) * 0.01;

      if (msg_->range.range < 1.0) {
        msg_->variance = kVarianceLow;
      } else {
        msg_->variance = kVarianceHigh;
      }
      cs_ += c;
      state_ = UsD1State::kSnr;
      break;
    }
    case UsD1State::kSnr: {
      msg_->snr = c;
      cs_ += c;
      state_ = UsD1State::kCs;
      break;
    }
    case UsD1State::kCs: {
      bool buffer_empty = (uart_->available() == 0);
      if ((c == (cs_ & 0xFF)) && buffer_empty) {
        if (publisher_) {
          publisher_->publish(msg_);
        }
        msg_->range.header.seq++;
      } else if (!buffer_empty) {
        if (nh_)
          nh_->logwarn("Rejecting US-D1 message. New available.");
      } else if (nh_) {
        nh_->logwarn("Rejecting US-D1 message. Wrong checksum.");
      }
      state_ = UsD1State::kHeader;
      break;
    }
    default: {
      state_ = UsD1State::kHeader;
      break;
    }
    }
  }
}
