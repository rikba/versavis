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

#include <cstdint>

class MeasurementState {
public:
  MeasurementState() {}

  void setMeasurement() {
    dr_ = true;
    num_++;
  }

  bool getDataReady(uint32_t *num) {
    if (dr_) {
      dr_ = false;
      if (num) {
        *num = num_;
      }
      return true;
    } else {
      return false;
    }
  }

  bool invert_ = false; // Signals whether the signal is inverted.

private:
  bool dr_ = false;           // Status whether measurement is ready.
  uint32_t num_ = 0xFFFFFFFF; // Measurement number.
};

#endif
