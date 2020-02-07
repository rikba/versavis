#include "sensors/Adis16448BmlzTriggered.h"

#include "helper.h"

Adis16448BmlzTriggered::Adis16448BmlzTriggered(TimerSynced *timer,
                                               const uint16_t rate_hz,
                                               const uint8_t dr_port_group,
                                               const uint8_t dr_pin,
                                               const uint8_t chip_select)
    : ImuSynced(timer), imu_(chip_select) {
  imu_.setup();

  if (timer_) {
    const bool kInvert = false;
    timer_->setupMfrq(rate_hz, kInvert);

    const auto dr_logic = InterruptLogic::kRise;
    timer_->setupDataReady(dr_port_group, dr_pin, dr_logic);
  }
}

void Adis16448BmlzTriggered::publish() {
  if (imu_msg_ && timer_ && timer_->isTriggered()) {
    imu_msg_->time.data = timer_->computeTimeLastTrigger();
    DEBUG_PRINTLN("triggered.");
  }

  if (timer_ && timer_->hasDataReady()) {
    int16_t *imu_data = imu_.sensorReadAllCRC();

    if (imu_msg_ && imu_.checksum(imu_data)) {
      imu_msg_->gx = imu_data[1];
      imu_msg_->gy = imu_data[2];
      imu_msg_->gz = imu_data[3];
      imu_msg_->ax = imu_data[4];
      imu_msg_->ay = imu_data[5];
      imu_msg_->az = imu_data[6];
      imu_msg_->mx = imu_data[7];
      imu_msg_->my = imu_data[8];
      imu_msg_->mz = imu_data[9];
      imu_msg_->baro = imu_data[10];
      imu_msg_->temp = imu_data[11];
      DEBUG_PRINTLN("publishing.");

      if (publisher_) {
        publisher_->publish(imu_msg_);
      }
    }
  }
}
