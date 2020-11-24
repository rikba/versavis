////////////////////////////////////////////////////////////////////////////////
//  March 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  MeasurementState.h
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Clock_Sync_MeasurementState_
#define Clock_Sync_MeasurementState_

#define BUFFER_SIZE 2
#include "helper.h"

#include <RingBufCPP.h>
#include <cstdint>
#include <ros/time.h>

#include "clock_sync/atomic.h"

enum SensorInterface { kSingleCapture = 0, kStrobe, kExternal };

// TODO(rikba): Add default values in C++14.
struct Measurement {
  uint32_t num;    // Measurement number.
  bool dr;         // Flag indicating whether data can be polled.
  ros::Time start; // Time measurement started.
  ros::Time stop;  // Time measurement ends.

  inline void computeStamp(ros::Time *stamp) const {
    auto half_exposure = stop - start;
    half_exposure *= 0.5;
    *stamp = start;
    (*stamp) += half_exposure;
  }
};

class MeasurementState {
public:
  MeasurementState() {}
  MeasurementState(const SensorInterface &new_measurement_indicator,
                   const SensorInterface &dr_indicator,
                   const SensorInterface &stamp_indicator,
                   const bool trigger_inverted, const bool dr_inverted,
                   const bool strobe_inverted)
      : trigger_inverted_(trigger_inverted), dr_inverted_(dr_inverted),
        strobe_inverted_(strobe_inverted),
        new_measurement_indicator_(new_measurement_indicator),
        dr_indicator_(dr_indicator), stamp_indicator_(stamp_indicator) {}

  MeasurementState(const SensorInterface &new_measurement_indicator,
                   const SensorInterface &dr_indicator,
                   const SensorInterface &stamp_indicator)
      : new_measurement_indicator_(new_measurement_indicator),
        dr_indicator_(dr_indicator), stamp_indicator_(stamp_indicator) {}

  // Create a new measurement by triggering or capturing a signal.
  void trigger(const ros::Time &time) {
    if (new_measurement_indicator_ == SensorInterface::kSingleCapture) {
      buffer_.add({.num = ++last_num_,
                   .dr = (dr_indicator_ == SensorInterface::kSingleCapture),
                   .start = time,
                   .stop = time},
                  true);
    }
  }

  // Create a new measurement by exposure start.
  void startStrobe(const ros::Time &time) {
    if (new_measurement_indicator_ == SensorInterface::kStrobe) {
      buffer_.add(
          {.num = ++last_num_, .dr = false, .start = time, .stop = time}, true);
    }
  }

  // Conclude measurement by exposure stop.
  void stopStrobe(const ros::Time &time) {
    if (buffer_.numElements()) {
      if (stamp_indicator_ == SensorInterface::kStrobe) {
        buffer_.peek(buffer_.numElements() - 1)->stop = time;
      }
      if (dr_indicator_ == SensorInterface::kStrobe) {
        buffer_.peek(buffer_.numElements() - 1)->dr = true;
      }
    }
  }

  // Conclude measurement by data ready signal.
  void setDataReady() {
    if (buffer_.numElements() &&
        (dr_indicator_ == SensorInterface::kExternal)) {
      buffer_.peek(buffer_.numElements() - 1)->dr = true;
    }
  }

  bool getMeasurement(Measurement *measurement) {
    // Find any measurement that has DR.
    bool has_measurement = false;
    bool ret = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for (size_t i = 0; i < buffer_.numElements(); i++) {
        if (buffer_.peek(i)->dr) {
          has_measurement = true;
          break;
        }
      }

      // Clean up old messages that did not get data ready signal.
      if (has_measurement) {
        while (buffer_.pull(measurement)) {
          if (measurement->dr) {
            ret = true;
            break;
          }
        }
      }
    }

    return ret;
  }

  inline void setNum(const uint32_t num) { last_num_ = num; }

  volatile bool trigger_inverted_ = false;
  volatile bool dr_inverted_ = false;
  volatile bool strobe_inverted_ = false;

private:
  RingBufCPP<struct Measurement, BUFFER_SIZE> buffer_;

  // Controls when a new measurement will be generated.
  SensorInterface new_measurement_indicator_ = SensorInterface::kSingleCapture;
  // Controls how the data ready flag is set.
  SensorInterface dr_indicator_ = SensorInterface::kSingleCapture;
  // Controls what signal stamps the data.
  SensorInterface stamp_indicator_ = SensorInterface::kSingleCapture;

  uint32_t last_num_ = 0xFFFFFFFF;
};

#endif
