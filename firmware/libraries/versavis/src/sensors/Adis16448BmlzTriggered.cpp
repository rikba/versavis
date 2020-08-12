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

void Adis16448BmlzTriggered::setupRos(char *frame_id, char *rate_topic,
                                      char *imu_topic, char *baro_topic,
                                      char *mag_topic, char *temp_topic) {
  ImuSynced::setupRos(rate_topic, imu_topic);

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

    // Assign publisher pointers.
    mag_pub_ = &mag_pub;
    baro_pub_ = &baro_pub;
    temp_pub_ = &temp_pub;

    // Advertise.
    nh_->advertise(*mag_pub_);
    nh_->advertise(*baro_pub_);
    nh_->advertise(*temp_pub_);
  }

  if (imu_msg_) {
    imu_msg_->header.frame_id = frame_id;
  }

  if (mag_msg_) {
    mag_msg_->header.frame_id = frame_id;
  }

  if (baro_msg_) {
    baro_msg_->header.frame_id = frame_id;
  }

  if (temp_msg_) {
    temp_msg_->header.frame_id = frame_id;
  }
}

bool Adis16448BmlzTriggered::publish() {
  bool new_measurement = false;

  if (timer_ && imu_msg_) {
    timer_->getTimeLastTrigger(&stamp_, &imu_msg_->header.seq);
  }

  if (imu_msg_ && timer_ && timer_->getDataReady(NULL)) {
    int16_t *imu_data = imu_.sensorReadAllCRC();

    if ((imu_.checksum(imu_data) == imu_data[12])) {
      new_measurement = true;
      bool has_mag_and_baro = imu_data[0] & (1 << 7);

      // BARO.
      if (has_mag_and_baro && baro_msg_) {
        baro_msg_->header.stamp = stamp_;
        baro_msg_->header.seq = imu_msg_->header.seq / 16;
        baro_msg_->fluid_pressure = imu_.pressureScale(imu_data[10]);
        baro_msg_->variance = -1.0;

        if (baro_pub_) {
          baro_pub_->publish(baro_msg_);
        }
      }

      // IMU.
      if (imu_msg_) {
        // TODO(rikba): Implement simple orientation filter.
        imu_msg_->header.stamp = stamp_;
        imu_msg_->header.stamp -= ros::Duration(0, IMU_DELAY_NS);
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

        // Calibrate gyro offset.
        switch (calibration_) {
        case CalibrationStatus::kInit: {
          smpl_prd_settings_ = imu_.regRead(SMPL_PRD);
          uint8_t avg = IMU_CALIBRATION_SAMPLES;
          uint16_t smpl_prd = (avg << 8) | (smpl_prd_settings_ & 0xFF);
          imu_.regWrite(SMPL_PRD, smpl_prd);
          nh_->loginfo("Start calibrating IMU bias.");
          calibration_ = CalibrationStatus::kRunning;
          break;
        }
        case CalibrationStatus::kRunning: {
          calibration_ = CalibrationStatus::kCalibrating;
          break;
        }
        case CalibrationStatus::kCalibrating: {
          nh_->loginfo("Calibrating IMU bias.");
          uint16_t xgyro_off = -imu_data[1];
          uint16_t ygyro_off = -imu_data[2];
          uint16_t zgyro_off = -imu_data[3];
          imu_.regWrite(XGYRO_OFF, xgyro_off);
          imu_.regWrite(YGYRO_OFF, ygyro_off);
          imu_.regWrite(ZGYRO_OFF, zgyro_off);
          calibration_ = CalibrationStatus::kResetAvg;
          break;
        }
        case CalibrationStatus::kResetAvg: {
          nh_->loginfo("Reset SMPL_PRD register.");
          imu_.regWrite(SMPL_PRD, smpl_prd_settings_);
          calibration_ = CalibrationStatus::kFinished;
          break;
        }
        case CalibrationStatus::kFinished: {
          // Toggle LED to visualize finished status.
          static uint32_t start_toggle = imu_msg_->header.seq;
          if ((imu_msg_->header.seq - start_toggle) > IMU_RATE) {
            start_toggle = imu_msg_->header.seq;
            uint16_t led_status = imu_.regRead(GPIO_CTRL);
            led_status ^= 1 << 9;
            imu_.regWrite(GPIO_CTRL, led_status);
          }
          break;
        }
        default: { break; }
        }
      }

      // MAG.
      if (has_mag_and_baro && mag_msg_) {
        mag_msg_->header.stamp = stamp_;
        mag_msg_->header.seq = imu_msg_->header.seq / 16;

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
    } else if (nh_) {
      nh_->logwarn("IMU checksum error.");
    }
  }

  return new_measurement;
}
