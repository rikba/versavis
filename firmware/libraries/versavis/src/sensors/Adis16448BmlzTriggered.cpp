#include "sensors/Adis16448BmlzTriggered.h"

Adis16448BmlzTriggered::Adis16448BmlzTriggered(ros::NodeHandle *nh,
                                               TimerSynced *timer,
                                               const uint16_t rate_hz,
                                               const uint8_t dr_port_group,
                                               const uint8_t dr_pin,
                                               const uint8_t chip_select)
    : ImuSynced(nh, timer), imu_(chip_select) {

  // Setup ADIS.
  imu_.setup();

  // Setup timers.
  if (timer_) {
    const bool kInvert = false;
    timer_->setupMfrq(rate_hz, kInvert);

    const auto dr_logic = InterruptLogic::kRise;
    timer_->setupDataReady(dr_port_group, dr_pin, dr_logic);
  }
}

void Adis16448BmlzTriggered::setupRos(const char *baro_topic,
                                      const char *imu_topic,
                                      const char *mag_topic,
                                      const char *temp_topic) {
  ImuSynced::setupRos(imu_topic);

  if (nh_) {
    // Create static ROS msgs.
    static sensor_msgs::FluidPressure baro_msg;
    static sensor_msgs::MagneticField mag_msg;
    static sensor_msgs::Temperature temp_msg;

    // Assign topic pointers.
    mag_msg_ = &mag_msg;
    baro_msg_ = &baro_msg;
    temp_msg_ = &temp_msg;

    // Create static ROS publishers.
    static ros::Publisher baro_pub(baro_topic, baro_msg_);
    static ros::Publisher mag_pub(mag_topic, mag_msg_);
    static ros::Publisher temp_pub(temp_topic, temp_msg_);
    static ros::Subscriber<std_msgs::UInt16, SensorSynced> rate_sub(
        "versavis/imu/change_rate", &SensorSynced::changeRateCb, this);

    // Assign publisher pointers.
    mag_pub_ = &mag_pub;
    baro_pub_ = &baro_pub;
    temp_pub_ = &temp_pub;
    rate_sub_ = &rate_sub;

    // Advertise.
    nh_->advertise(*mag_pub_);
    nh_->advertise(*baro_pub_);
    nh_->advertise(*temp_pub_);
    nh_->subscribe(*rate_sub_);
  }

  if (imu_msg_) {
    imu_msg_->header.frame_id = "adis16448";
  }

  if (mag_msg_) {
    mag_msg_->header.frame_id = "adis16448";
  }

  if (baro_msg_) {
    baro_msg_->header.frame_id = "adis16448";
  }

  if (temp_msg_) {
    temp_msg_->header.frame_id = "adis16448";
  }
}

void Adis16448BmlzTriggered::publish() {
  if (timer_ && imu_msg_) {
    timer_->getTimeLastTrigger(&imu_msg_->header.stamp, &imu_msg_->header.seq);
  }

  if (imu_msg_ && timer_ && timer_->getDataReady(NULL)) {
    int16_t *imu_data = imu_.sensorReadAllCRC();

    if ((imu_.checksum(imu_data) == imu_data[12])) {

      // MAG and baro update every 16th trigger.
      // We are not quite sure when the first trigger happens, so we calibrate
      // the offset.
      bool has_mag_and_baro = (mag_baro_offset_ != 0xFF);
      if (!has_mag_and_baro) {
        if (prev_pressure == 0xFFFF) {
          prev_pressure = imu_data[10];
        } else if (prev_pressure != imu_data[10]) {
          mag_baro_offset_ = imu_msg_->header.seq % 16;
        }
      } else {
        // https://ez.analog.com/mems/w/documents/4122/adis16448-data-sampling
        // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, -->15<--
        has_mag_and_baro &=
            ((imu_msg_->header.seq - mag_baro_offset_) % 16) == 0;
      }

      // BARO.
      if (has_mag_and_baro && baro_msg_) {
        baro_msg_->header.stamp = imu_msg_->header.stamp;
        baro_msg_->header.seq = (imu_msg_->header.seq - mag_baro_offset_) / 16;
        baro_msg_->fluid_pressure = imu_.pressureScale(imu_data[10]);
        baro_msg_->variance = -1.0;

        if (baro_pub_) {
          baro_pub_->publish(baro_msg_);
        }
      }

      // IMU.
      if (imu_msg_) {
        // TODO(rikba): Implement simple orientation filter.
        imu_msg_->orientation_covariance[0] = -1.0;

        imu_msg_->angular_velocity.x = imu_.gyroScale(imu_data[1]);
        imu_msg_->angular_velocity.y = imu_.gyroScale(imu_data[2]);
        imu_msg_->angular_velocity.z = imu_.gyroScale(imu_data[3]);
        // TODO(rikba): Determine covariance, e.g., Allan deviation.
        imu_msg_->angular_velocity_covariance[0] = 6e-9;
        imu_msg_->angular_velocity_covariance[4] = 6e-9;
        imu_msg_->angular_velocity_covariance[8] = 6e-9;

        imu_msg_->linear_acceleration.x = imu_.accelScale(imu_data[4]);
        imu_msg_->linear_acceleration.y = imu_.accelScale(imu_data[5]);
        imu_msg_->linear_acceleration.z = imu_.accelScale(imu_data[6]);
        imu_msg_->linear_acceleration_covariance[0] = 0.043864908;
        imu_msg_->linear_acceleration_covariance[4] = 0.043864908;
        imu_msg_->linear_acceleration_covariance[8] = 0.043864908;

        if (publisher_) {
          publisher_->publish(imu_msg_);
        }
      }

      // MAG.
      if (has_mag_and_baro && mag_msg_) {
        mag_msg_->header.stamp = imu_msg_->header.stamp;
        mag_msg_->header.seq = (imu_msg_->header.seq - mag_baro_offset_) / 16;

        mag_msg_->magnetic_field.x = imu_.magnetometerScale(imu_data[7]);
        mag_msg_->magnetic_field.y = imu_.magnetometerScale(imu_data[8]);
        mag_msg_->magnetic_field.z = imu_.magnetometerScale(imu_data[9]);
        mag_msg_->magnetic_field_covariance[0] = -1.0;

        if (mag_pub_) {
          mag_pub_->publish(mag_msg_);
        }
      }

      // TEMP.
      if (temp_msg_) {
        temp_msg_->header = imu_msg_->header;
        temp_msg_->temperature = imu_.tempScale(imu_data[11]);
        temp_msg_->variance = -1.0;

        if (temp_pub_) {
          temp_pub_->publish(temp_msg_);
        }
      }
    }
  }
}
