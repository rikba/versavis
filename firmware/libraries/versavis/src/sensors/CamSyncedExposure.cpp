#include "sensors/CamSyncedExposure.h"

CamSyncedExposure::CamSyncedExposure(TccSynced *timer, const uint16_t rate_hz,
                                     const bool invert_trigger,
                                     const bool invert_exposure)
    : CamSynced(timer, rate_hz, invert_trigger) {

  // Setup timers.
  if (timer) {
    timer->setupExposure(invert_exposure);
  }
}

void CamSyncedExposure::publish() {
  uint32_t num = 0;
  if (timer_ && time_msg_ &&
      static_cast<TccSynced *>(timer_)->computeTimeLastMidExposure(
          &time_msg_->time, NULL, &num)) {
    if (publisher_) {
      time_msg_->number = static_cast<uint64_t>(num);
      publisher_->publish(time_msg_);
    }
  }
}
