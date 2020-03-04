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

  bool getDataReady(volatile uint32_t *num) {
    // Savely copy state.
    auto dr_cpy = dr_;
    dr_ = false;
    auto num_cpy = num_;

    if (dr_cpy) {
      if (num) {
        *num = num_cpy;
      }
      return true;
    } else {
      return false;
    }
  }

  volatile bool invert_ = false; // Signals whether the signal is inverted.

private:
  volatile bool dr_ = false;           // Status whether measurement is ready.
  volatile uint32_t num_ = 0xFFFFFFFF; // Measurement number.
};

#endif
