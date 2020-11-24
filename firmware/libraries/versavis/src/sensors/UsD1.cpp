#include "sensors/UsD1.h"

#include "clock_sync/RtcSync.h"

const uint8_t kHeader = 0xFE;
const uint8_t kVersionId = 0x02;

UsD1::UsD1(ros::NodeHandle *nh, Uart *uart) : nh_(nh), uart_(uart) {
  if (uart_) {
    uart_->begin(115200);
  }
}

void UsD1::setupRos(const char *data_topic) {
  if (nh_) {
    // Create static ROS msg.
    static versavis::UsD1Micro msg;

    // Assign topic pointer.
    msg_ = &msg;

    // Create static ROS publisher.
    static ros::Publisher pub(data_topic, msg_);

    // Assign publisher pointers.
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(*publisher_);
  }
}

bool UsD1::publish() {
  bool new_char = false;

  if (msg_ && uart_ && uart_->available()) {
    new_char = true;
    uint8_t c = uart_->read();

    // Parse character.
    switch (state_) {
    case UsD1State::kHeader: {
      if (c == kHeader) {
        msg_->time.data = RtcSync::getInstance().getTimeNow();
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
      msg_->range = ((c << 8) + lsb_);
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
        msg_->number++;
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

  return new_char;
}
