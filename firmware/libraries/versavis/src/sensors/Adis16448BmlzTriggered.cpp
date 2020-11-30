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
    timer_->setMeasurementState(
        {SensorInterface::kSingleCapture, SensorInterface::kExternal,
         SensorInterface::kSingleCapture, false, false, false, true});
    timer_->setupMfrq(rate_hz);

    const auto dr_logic = InterruptLogic::kRise;
    timer_->setupDataReady(dr_port_group, dr_pin, dr_logic);
  }
}

void Adis16448BmlzTriggered::setupRos(const char *rate_topic,
                                      const char *imu_topic,
                                      const char *baro_topic,
                                      const char *mag_topic,
                                      const char *temp_topic) {
  ImuSynced::setupRos(rate_topic, imu_topic);

  if (nh_) {
    // Create static ROS msgs.
    static versavis::PressureMicro baro_msg;
    static versavis::MagneticMicro mag_msg;
    static versavis::TemperatureMicro temp_msg;

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
}

bool Adis16448BmlzTriggered::read() {
  bool new_measurement = false;

  if (imu_msg_ && timer_ && timer_->getMeasurement(&measurement_)) {
    int16_t *imu_data = imu_.sensorReadAllCRC();
    imu_msg_->number = measurement_.num;

    if ((imu_.checksum(imu_data) == imu_data[12])) {
      new_measurement = true;
      bool has_mag_and_baro = imu_data[0] & (1 << 7);

      // BARO.
      if (has_mag_and_baro && baro_msg_) {
        baro_msg_->time.data = measurement_.start;
        baro_msg_->number = imu_msg_->number / 16;
        baro_msg_->pressure = imu_data[10];

        if (!baro_buffer_.add(*baro_msg_, true) && nh_) {
          nh_->logwarn("Baro buffer full.");
        }
      }

      // IMU.
      if (imu_msg_) {
        // TODO(rikba): Implement simple orientation filter.
        imu_msg_->time.data = measurement_.start;

        imu_msg_->gx = imu_data[1];
        imu_msg_->gy = imu_data[2];
        imu_msg_->gz = imu_data[3];

        imu_msg_->ax = imu_data[4];
        imu_msg_->ay = imu_data[5];
        imu_msg_->az = imu_data[6];

        if (!imu_buffer_.add(*imu_msg_, true) && nh_) {
          nh_->logwarn("IMU buffer full.");
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
          static uint32_t start_toggle = imu_msg_->number;
          if ((imu_msg_->number - start_toggle) > IMU_RATE) {
            start_toggle = imu_msg_->number;
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
        mag_msg_->time.data = measurement_.start;
        mag_msg_->number = imu_msg_->number / 16;

        mag_msg_->mx = imu_data[7];
        mag_msg_->my = imu_data[8];
        mag_msg_->mz = imu_data[9];

        if (!mag_buffer_.add(*mag_msg_, true) && nh_) {
          nh_->logwarn("Mag buffer full.");
        }
      }

      // TEMP.
      if (temp_msg_) {
        temp_msg_->time.data = measurement_.start;
        temp_msg_->temperature = imu_data[11];

        if (!temp_buffer_.add(*temp_msg_, true) && nh_) {
          nh_->logwarn("Temp buffer full.");
        }
      }
    } else if (nh_) {
      // nh_->logwarn("IMU checksum error.");
    }
  }

  return new_measurement;
}

bool Adis16448BmlzTriggered::publish() {

  static versavis::ImuMicro imu;
  bool publish_imu = imu_buffer_.pull(&imu);
  if (publish_imu && publisher_) {
    publisher_->publish(&imu);
  }

  static versavis::PressureMicro baro;
  bool publish_baro = baro_buffer_.pull(&baro);
  if (publish_baro && baro_pub_) {
    baro_pub_->publish(&baro);
  }

  static versavis::MagneticMicro mag;
  bool publish_mag = mag_buffer_.pull(&mag);
  if (publish_mag && mag_pub_) {
    mag_pub_->publish(&mag);
  }

  static versavis::TemperatureMicro temp;
  bool publish_temp = temp_buffer_.pull(&temp);
  if (publish_temp && temp_pub_) {
    temp_pub_->publish(&temp);
  }

  return publish_imu || publish_baro || publish_mag || publish_temp;
}
