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

    compensateExposure();
  }

  return new_measurement;
}

// Every change in second check how far off the time stamp was from the exact
// second. Compensate for this offset.
void CamSyncedExposure::compensateExposure() {
  if (timer_ && (img_msg_->stamp.sec == last_stamp_.sec + 1)) {
    // Seconds incremented.
    // Check which time stamp is closer to the full second.
    ros::Time this_second = ros::Time(img_msg_->stamp.sec, 0);
    auto d1 = (this_second - img_msg_->stamp).toSec();
    auto d2 = (this_second - last_stamp_).toSec();
    double d;
    if (fabs(d1) < fabs(d2)) {
      timer_->offsetTrigger(d1);
      d = d1;
    } else {
      timer_->offsetTrigger(d2);
      d = d2;
    }
    if (nh_) {
      char info[50];
      sprintf(info, "Offset compensation: %.9f", d);
      nh_->loginfo(info);
    }
  }

  last_stamp_ = img_msg_->stamp;
}

void CamSyncedExposure::setSeqCb(const std_msgs::UInt32 &seq_msg) {
  if (timer_) {
    static_cast<TccSynced *>(timer_)->setExposureStateNum(seq_msg.data);
    if (nh_) {
      nh_->loginfo("Setting exposure header sequence.");
    }
  }
}
