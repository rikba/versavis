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

inline ros::Duration computeDuration(const ros::Time &start,
                                     const ros::Time &stop) {
  int32_t sec = stop.sec - start.sec;
  int32_t nsec = stop.nsec - start.nsec;
  return ros::Duration(sec, nsec);
}

class MeasurementStateExposure : public MeasurementStateStamped {
public:
  MeasurementStateExposure() {}

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
    // Savely copy state.
    ros::Duration exposure_cpy = {exposure_.sec, exposure_.nsec};

    if (MeasurementStateStamped::getTime(time, num)) {
      if (exp) {
        *exp = exposure_cpy;
      }
      return true;
    } else {
      return false;
    }
  }

private:
  ros::Time start_time_;
  volatile ros::Duration exposure_;
};

#endif
