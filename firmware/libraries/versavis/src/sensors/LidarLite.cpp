#include "sensors/LidarLite.h"

#include <Wire.h>

const uint8_t kAddress = 0x62;
const bool kStopTransmission = true;

LidarLite::LidarLite(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz)
    : RangeSynced(nh, timer) {
  // Setup MODE_PIN to trigger measurements.
  if (timer_) {
    const bool kInvert = true; // Change from high to low triggers measurement.
    timer_->setupMfrq(rate_hz, kInvert);
  }

  if (range_msg_) {
    last_msg_ = range_msg_->header.seq;
  }

  // // Configure I2C sensor.
  // Wire.begin();
  // write(0x00, 0x00); // Default reset.
}

void LidarLite::publish() {
  // Obtain new stamp after triggering.
  if (timer_ && range_msg_) {
    timer_->getTimeLastTrigger(&range_msg_->header.stamp,
                               &range_msg_->header.seq);
  }

  // if (busy()) {
  //   nh_->loginfo("busy");
  // } else {
  //   nh_->loginfo("not busy");
  // }

  // If we have a new message number, try to obtain range message from I2C.
  if (range_msg_ && (last_msg_ != range_msg_->header.seq) && !busy()) {
    // // Read range measurement.
    // uint16_t range_cm;
    // auto nack = readDistance(&range_cm);

    // if (nack == 0) {
    //
    //   range_msg_->range = static_cast<float>(range_cm / 100);
    //   range_msg_->range += static_cast<float>(range_cm % 100) * 0.01;

    // ROS publish.
    if (publisher_) {
      publisher_->publish(range_msg_);
    }
    // }

    // Update state.
    last_msg_ = range_msg_->header.seq;
  }
}

uint8_t LidarLite::readDistance(uint16_t *distance) const {
  Wire.beginTransmission(kAddress);
  Wire.write(0x8F);
  uint8_t nack = Wire.endTransmission(kStopTransmission);
  Wire.requestFrom(kAddress, 2, kStopTransmission);

  uint8_t high = Wire.read();
  uint8_t low = Wire.read();

  *distance = (high << 8) + low;
  return nack;
}

uint8_t LidarLite::write(uint8_t reg_adr, uint8_t data) const {
  Wire.beginTransmission(kAddress);
  Wire.write(reg_adr);
  Wire.write(data);
  return Wire.endTransmission(kStopTransmission);
}

bool LidarLite::busy() const {
  Wire.beginTransmission(kAddress);
  Wire.write(0x01);
  uint8_t nack = Wire.endTransmission(kStopTransmission);
  Wire.requestFrom(kAddress, 1, kStopTransmission);
  uint8_t status = Wire.read();
  return (nack == 0) && (status & 0b1);
}
