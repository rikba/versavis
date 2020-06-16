#include "sensors/CamSyncedExposure.h"

CamSyncedExposure::CamSyncedExposure(ros::NodeHandle *nh, TccSynced *timer,
                                     const uint16_t rate_hz,
                                     const bool invert_trigger,
                                     const bool invert_exposure)
    : CamSynced(nh, timer, rate_hz, invert_trigger) {

  // Setup timers.
  if (timer) {
    timer->setupExposure(invert_exposure);
  }
}

bool CamSyncedExposure::publish() {
  uint32_t num = 0;
  bool new_measurement = false;
  if (timer_ && time_msg_ &&
      static_cast<TccSynced *>(timer_)->getTimeLastExposure(&time_msg_->time,
                                                            &num, NULL)) {
    new_measurement = true;
    if (publisher_) {
      time_msg_->number = static_cast<uint64_t>(num);
      publisher_->publish(time_msg_);
    }
  }

  return new_measurement;
}
