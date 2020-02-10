#include "sensors/Adis16448BmlzTriggered.h"

Adis16448BmlzTriggered::Adis16448BmlzTriggered(TimerSynced *timer,
                                               const uint16_t rate_hz,
                                               const uint8_t dr_port_group,
                                               const uint8_t dr_pin,
                                               const uint8_t chip_select)
    : ImuSynced(timer), imu_(chip_select) {

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

void Adis16448BmlzTriggered::setupRos(ros::NodeHandle *nh,
                                      const char *baro_topic,
                                      const char *imu_topic,
                                      const char *mag_topic,
                                      const char *temp_topic) {
  ImuSynced::setupRos(nh, imu_topic);

  if (nh) {
    // Create static ROS msgs.
    static versavis::MagneticMicro mag_msg;
    static versavis::PressureMicro baro_msg;
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
    nh->advertise(*mag_pub_);
    nh->advertise(*baro_pub_);
    nh->advertise(*temp_pub_);
  }
}

void Adis16448BmlzTriggered::publish() {
  if (imu_msg_ && timer_ && timer_->isTriggered()) {
    imu_msg_->time.data = timer_->computeTimeLastTrigger();
    mag_msg_->time.data = imu_msg_->time.data;
    baro_msg_->time.data = imu_msg_->time.data;
    temp_msg_->time.data = imu_msg_->time.data;
  }

  if (timer_ && timer_->hasDataReady()) {
    int16_t *imu_data = imu_.sensorReadAllCRC();

    if ((imu_.checksum(imu_data) == imu_data[12])) {
      // BARO.
      if (baro_msg_) {
        baro_msg_->pressure = imu_data[10];

        if (baro_pub_) {
          baro_pub_->publish(baro_msg_);
        }
      }

      // IMU.
      if (imu_msg_) {
        imu_msg_->gx = imu_data[1];
        imu_msg_->gy = imu_data[2];
        imu_msg_->gz = imu_data[3];
        imu_msg_->ax = imu_data[4];
        imu_msg_->ay = imu_data[5];
        imu_msg_->az = imu_data[6];

        if (publisher_) {
          publisher_->publish(imu_msg_);
        }
      }

      // MAG.
      if (mag_msg_) {
        mag_msg_->mx = imu_data[7];
        mag_msg_->my = imu_data[8];
        mag_msg_->mz = imu_data[9];

        if (mag_pub_) {
          mag_pub_->publish(mag_msg_);
        }
      }

      // TEMP.
      if (temp_msg_) {
        temp_msg_->temperature = imu_data[11];

        if (temp_pub_) {
          temp_pub_->publish(temp_msg_);
        }
      }
    }
  }
}
