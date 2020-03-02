////////////////////////////////////////////////////////////////////////////////
//  March 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  MeasurementStateStamped.h
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Clock_Sync_MeasurementStateStamped_
#define Clock_Sync_MeasurementStateStamped_

#include <cstdint>
#include <ros/time.h>

#include "clock_sync/MeasurementState.h"

class MeasurementStateStamped : public MeasurementState {
public:
  MeasurementStateStamped() {}

  inline void setTime(const ros::Time &time) {
    setMeasurement();
    time_ = time;
  }

  inline bool getTime(ros::Time *time, uint32_t *num) {
    if (getDataReady(num)) {
      if (time) {
        *time = time_;
      }
      return true;
    } else {
      return false;
    }
  }

private:
  ros::Time time_;
};

#endif
