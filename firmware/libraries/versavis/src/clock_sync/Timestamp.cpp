#include "clock_sync/Timestamp.h"

#ifdef USE_GNSS_TIME
#include "clock_sync/GnssSync.h"
#endif

bool Timestamp::getTime(ros::Time *time) {
  bool success = has_time_;
  // Get time.
  if (time)
    *time = time_;

  // Reset state.
  has_time_ = false;
  time_.sec = 0;
  time_.nsec = 0;

  return success;
}

void Timestamp::setTimeNow() {
#ifdef USE_GNSS_TIME
  time_ = GnssSync::getInstance().getTimeNow();
#else
  if (nh_)
    time_ = nh_->now();
#endif
  has_time_ = true;
}
