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
  TccSynced(Tcc *tcc);

  void setupPwm(uint16_t rate_hz, uint32_t pulse_us, bool invert);
  void setupDataReady(const uint8_t port_group, const uint8_t pin,
                      const InterruptLogic &logic) override {}
  void setupMfrq(const uint16_t rate_hz, const bool invert) override {}

  void handleInterrupt() override;

private:
  // Pointer to the actual timer.
  Tcc *tcc_ = NULL;

  void setup() const;
};

#endif