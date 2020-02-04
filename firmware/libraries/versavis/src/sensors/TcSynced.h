////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TcSynced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef TcSynced_h
#define TcSynced_h

#include <Arduino.h>

#include "sensors/TimerSynced.h"

class TcSynced : public TimerSynced {
public:
  TcSynced(TcCount16 *tc);

  void setup() const override;

  void handleInterrupt() override;

private:
  // Pointer to the actual timer.
  TcCount16 *tc_ = NULL;
};

#endif
