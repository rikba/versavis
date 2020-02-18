////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TccSynced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef TccSynced_h
#define TccSynced_h

#include <Arduino.h>

#include "clock_sync/TimerSynced.h"

class TccSynced : public TimerSynced {
public:
  TccSynced(const MfrqPin &mfrq_pin, Tcc *tcc);

  void setupDataReady(const uint8_t port_group, const uint8_t pin,
                      const InterruptLogic &logic) override {}
  void setupMfrqWaveform() const override;

  void handleInterrupt() override;

private:
  // Pointer to the actual timer.
  Tcc *tcc_ = NULL;

  void setup() const;
};

#endif
