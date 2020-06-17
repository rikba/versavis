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
  void setMeasurement() {
    dr_ = true;
    num_++;
  }

  bool getDataReady(volatile uint32_t *num) {
    // Savely invalidate data ready flag and return state.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
  }

  inline void setNum(const uint32_t num) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { num_ = num; }
  }

  volatile bool invert_ = false; // Signals whether the signal is inverted.

private:
  volatile bool dr_ = false;           // Status whether measurement is ready.
  volatile uint32_t num_ = 0xFFFFFFFF; // Measurement number.
};

#endif
