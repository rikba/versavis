#include "sensors/CamSyncedExposure.h"

CamSyncedExposure::CamSyncedExposure(ros::NodeHandle *nh, TccSynced *timer,
                                     const uint16_t rate_hz,
                                     const bool invert_trigger,
                                     const bool invert_exposure,
                                     const bool exposure_compensation)
    : CamSynced(nh, timer, rate_hz, invert_trigger),
      exposure_compensation_(exposure_compensation) {

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
    if (exposure_compensation_) {
      compensateExposure();
    }
    new_measurement = true;
    if (publisher_) {
      publisher_->publish(img_msg_);
    }
  }

  return new_measurement;
}

// Check how far off the time stamp was from the exact second. Compensate for
// this offset.
void CamSyncedExposure::compensateExposure() {
  // TODO(rikba): This breaks if exposure is not finished before next trigger.
  // It would be more robust to find a mapping between trigger and exposure.
  // For now we check the last two nominal time stamps.
  prev_expected_stamp_ = expected_stamp_;
  if (timer_) {
    timer_->getTimeLastNominalTrigger(&expected_stamp_, NULL);
  }
  if (timer_ && prev_expected_stamp_.sec) {
    // Check which time stamp is closer to the full second.
    auto d1 = (expected_stamp_ - img_msg_->stamp).toSec();
    auto d2 = (prev_expected_stamp_ - img_msg_->stamp).toSec();
    if (fabs(d1) < fabs(d2)) {
      timer_->offsetTrigger(d1);
    } else {
      timer_->offsetTrigger(d2);
    }
  }
}

void CamSyncedExposure::setSeqCb(const std_msgs::UInt32 &seq_msg) {
  if (timer_) {
    static_cast<TccSynced *>(timer_)->setExposureStateNum(seq_msg.data);
    if (nh_) {
      nh_->loginfo("Setting exposure header sequence.");
    }
  }
}
