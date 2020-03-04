////////////////////////////////////////////////////////////////////////////////
//  March 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  MeasurementStateExposure.h
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Clock_Sync_MeasurementStateExposure_
#define Clock_Sync_MeasurementStateExposure_

#include <cstdint>
#include <ros/duration.h>
#include <ros/time.h>

#include "clock_sync/MeasurementStateStamped.h"
#include "clock_sync/atomic.h"

inline ros::Duration computeDuration(const ros::Time &start,
                                     const ros::Time &stop) {
  return ros::Duration(stop.sec - start.sec, stop.nsec - start.nsec);
}

class MeasurementStateExposure : public MeasurementStateStamped {
public:
  inline void setStart(const ros::Time &time) { start_time_ = time; }
  inline void setEnd(const ros::Time &time) {
    if ((time.sec > start_time_.sec) ||
        ((time.sec == start_time_.sec) && (time.nsec >= start_time_.nsec))) {
      auto half_exposure = computeDuration(start_time_, time);
      exposure_.sec = half_exposure.sec;
      exposure_.nsec = half_exposure.nsec;
      half_exposure *= 0.5;
      start_time_ += half_exposure;
      setTime(start_time_);
    }
  }

  inline bool getTime(ros::Time *time, uint32_t *num, ros::Duration *exp) {
    // Savely return state.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (MeasurementStateStamped::getTime(time, num)) {
        if (exp) {
          exp->sec = exposure_.sec;
          exp->nsec = exposure_.nsec;
        }
        return true;
      } else {
        return false;
      }
    }
  }

private:
  ros::Time start_time_;
  volatile ros::Duration exposure_;
};

#endif
