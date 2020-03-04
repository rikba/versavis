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

#include "clock_sync/atomic.h"

class MeasurementState {
public:
  MeasurementState() {}

  void setMeasurement() {
    dr_ = true;
    num_++;
  }

  bool getDataReady(volatile uint32_t *num) {
    // Savely copy state and invalidate data ready flag.
    bool dr_cpy;
    uint32_t num_cpy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      dr_cpy = dr_;
      dr_ = false;
      num_cpy = num_;
    }

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
