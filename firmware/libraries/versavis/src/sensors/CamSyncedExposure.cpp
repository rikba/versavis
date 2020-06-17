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
  bool new_measurement = false;
  if (timer_ && img_msg_ &&
      static_cast<TccSynced *>(timer_)->getTimeLastExposure(
          &img_msg_->stamp, &img_msg_->seq, NULL)) {
    new_measurement = true;
    if (publisher_) {
      publisher_->publish(img_msg_);
    }
  }

  return new_measurement;
}
