#include "sensors/CamSyncedExposure.h"

CamSyncedExposure::CamSyncedExposure(ros::NodeHandle *nh, TccSynced *timer,
                                     const uint16_t rate_hz,
                                     const bool invert_trigger,
                                     const bool invert_exposure,
                                     const bool exposure_compensation)
    : CamSynced(nh, timer, rate_hz,
                {SensorInterface::kStrobe, SensorInterface::kStrobe,
                 SensorInterface::kStrobe, invert_trigger, false,
                 invert_exposure}),
      exposure_compensation_(exposure_compensation) {

  // Setup timers.
  if (timer) {
    timer->setupExposure();
  }
}

bool CamSyncedExposure::publish() {
  bool new_measurement = false;

  if (timer_ && img_msg_ && timer_->getMeasurement(&measurement_)) {
    prev_stamp_ = img_msg_->stamp;
    measurement_.computeStamp(&img_msg_->stamp);
    img_msg_->seq = measurement_.num;

    if (publisher_) {
      publisher_->publish(img_msg_);
    }
    if (exposure_compensation_) {
      compensateExposure();
    }
    new_measurement = true;
  }

  return new_measurement;
}

// Check how far off the time stamp was from the exact second. Compensate for
// this offset.
void CamSyncedExposure::compensateExposure() {
  if (timer_ && img_msg_ && prev_stamp_.sec &&
      ((prev_stamp_.sec + 1) == img_msg_->stamp.sec)) {
    // Check which time stamp is closer to the full second.
    auto d1 =
        (img_msg_->stamp.nsec < 5e8)
            ? (ros::Time(img_msg_->stamp.sec, 0) - img_msg_->stamp).toSec()
            : (ros::Time(img_msg_->stamp.sec + 1, 0) - img_msg_->stamp).toSec();
    auto d2 = (prev_stamp_.nsec < 5e8)
                  ? (ros::Time(prev_stamp_.sec, 0) - prev_stamp_).toSec()
                  : (ros::Time(prev_stamp_.sec + 1, 0) - prev_stamp_).toSec();

    if (fabs(d1) < fabs(d2)) {
      timer_->offsetTrigger(d1);
    } else {
      timer_->offsetTrigger(d2);
    }
  }
}
