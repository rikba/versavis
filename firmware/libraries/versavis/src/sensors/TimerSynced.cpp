#include "sensors/TimerSynced.h"

#include "clock_sync/RtcSync.h"

TimerSynced::TimerSynced() {
  RtcSync::getInstance(); // Make sure RTC singleton exists.
}

void TimerSynced::sync() {
  // Get total seconds from RTC.
  if (sync_rtc_stamp_) {
    stamp_ = ros::Time(RtcSync::getInstance().getSecs(), 0);
    sync_rtc_stamp_ = false;
  }
}
