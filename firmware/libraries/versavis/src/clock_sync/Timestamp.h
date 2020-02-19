////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Timestamp.h
////////////////////////////////////////////////////////////////////////////////
//
//  This class handles timestamping using the RTC clock. A time stamp is always
//  counted from the last RTC second. syncRtc() and overflow() need to be called
//  for each timestamp in the timer interrupt handlers. A timestamp can only be
//  requested once and needs to be requested before the next RTC triggering or
//  overflow.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Timestamp_h
#define Timestamp_h

#include <cstdint>

#include "clock_sync/RtcSync.h"
#include <ros/duration.h>
#include <ros/time.h>

class Timestamp {
public:
  Timestamp() {}
  inline void syncRtc() {
    rtc_secs_ = RtcSync::getInstance().getSecs();
    ovf_counter_ = 0;
    has_time_ = false; // Invalidate.
    // TODO(rikba): Instead of invalidating timestamp one could consider
    // buffering it.
  }

  inline void overflow() {
    ovf_counter_++;
    has_time_ = false; // Invalidate.
    // TODO(rikba): Instead of invalidating timestamp one could consider
    // buffering it.
  }

  inline void setTicks(const uint32_t cc) {
    cc_ = cc;
    has_time_ = true;
  }

  inline bool computeTime(const uint8_t prescaler, const uint32_t top,
                          ros::Time *time) {
    if (has_time_) {
      has_time_ = false; // Invalidate.
      uint32_t ticks = cc_;
      // TODO(rikba): ovf_counter_ can potentially change while computing ticks.
      // This would result in an invalid time. Maybe atomic helps?
      ticks += ovf_counter_ * (top + 1); // Account for overflows.
      if (time) {
        *time = RtcSync::getInstance().computeTime(rtc_secs_, ticks,
                                                   kPrescalers[prescaler]);
      }
      return true;
    } else {
      if (time) {
        *time = ros::Time();
      }
      return false;
    }
  }

private:
  uint32_t cc_ = 0xFFFFFFFF;          // Captured timer ticks.
  uint32_t rtc_secs_ = 0xFFFFFFFF;    // RTC seconds at moment of time stamp.
  uint32_t ovf_counter_ = 0xFFFFFFFF; // Overflows since last second.

  bool has_time_ = false; // Indicate if time is available to compute.
};

inline ros::Duration computeDuration(const ros::Time &start,
                                     const ros::Time &stop) {
  int32_t sec = stop.sec - start.sec;
  int32_t nsec = stop.nsec - start.nsec;
  return ros::Duration(sec, nsec);
}

#endif
