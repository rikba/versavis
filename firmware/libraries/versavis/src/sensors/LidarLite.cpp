#include "sensors/LidarLite.h"

#include <Wire.h>

const uint8_t kAddress = 0x62;
const bool kStopTransmission = true;
const float kVarianceLow = pow(0.025 / 3, 2);
const float kVarianceHigh = pow(0.1 / 3, 2);

LidarLite::LidarLite(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz)
    : SensorSynced(nh, timer) {
  // Setup MODE_PIN to trigger measurements.
  if (timer_) {
    const bool kInvert = true; // Change from high to low triggers measurement.
    const uint16_t kPulseUs = 40;
    timer_->setupMpwm(rate_hz, kPulseUs, kInvert);
  }

  // Configure I2C sensor.
  Wire.begin();
  Wire.setClock(3400000); // High speed mode.
  write(0x00, 0x00);      // Default reset.
}

void LidarLite::setupRos(const char *topic) {
  if (nh_) {
    // Create static ROS msg.
    static versavis::LidarLite msg;

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
    msg_->range.header.frame_id = "LidarLite";

    // https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
    msg_->range.radiation_type = sensor_msgs::Range::INFRARED;
    msg_->range.field_of_view = 0.008;
    msg_->range.min_range = 0.05;
    msg_->range.max_range = 30.0;

    last_msg_ = msg_->range.header.seq;
  }
}

bool LidarLite::publish() {
  bool new_measurement = false;

  // Obtain new stamp after triggering.
  if (timer_ && msg_) {
    timer_->getTimeLastTrigger(&msg_->range.header.stamp,
                               &msg_->range.header.seq);
  }

  // If we have a new message number, try to obtain range message from I2C.
  if (msg_ && (last_msg_ != msg_->range.header.seq) && !busy()) {
    // Read measurement.
    if (readData(msg_) && publisher_) {
      publisher_->publish(msg_);
    }

    // Update state.
    new_measurement = true;
    last_msg_ = msg_->range.header.seq;
  }

  return new_measurement;
}

bool LidarLite::readData(versavis::LidarLite *msg) const {
  Wire.beginTransmission(kAddress);
  Wire.write(0x8E); // Start from signal strength with auto-increment bit
  uint8_t nack = Wire.endTransmission(kStopTransmission);
  if (nack && nh_) {
    nh_->logwarn("NACK: Failed to read Lidar Lite data.");
  }
  Wire.requestFrom(kAddress, 3, kStopTransmission);

  if (msg) {
    // Info
    msg->signal_strength = Wire.read();

    // Range
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();

    msg->range.range = ((high << 8) + low) * 0.01;

    // Variance
    if (msg->range.range < 5.0) {
      msg->variance = kVarianceLow;
    } else {
      msg->variance = kVarianceHigh;
    }
  }
  return (nack == 0);
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
  if (nack && nh_) {
    nh_->logwarn("NACK: Failed to check Lidar Lite status.");
  }
  Wire.requestFrom(kAddress, 1, kStopTransmission);
  uint8_t status = Wire.read();
  return (nack == 0) && (status & 0b1);
}
