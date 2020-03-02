#include "clock_sync/Tc4Synced.h"

// Setup MFRQ pin.
// TODO(rikba)
#define TC4_MFRQ_GROUP 255
#define TC4_MFRQ_PIN 255
#define TC4_MFRQ_DRVSTR 0

Tc4Synced::Tc4Synced()
    : TcSynced(MfrqPin{.group = TC4_MFRQ_GROUP,
                       .pin = TC4_MFRQ_PIN,
                       .drvstr = TC4_MFRQ_DRVSTR},
               (TcCount16 *)TC4) {
  // Enable interrupts.
  NVIC_SetPriority(TC4_IRQn, 0);
  NVIC_EnableIRQ(TC4_IRQn);
}

void TC4_Handler() { Tc4Synced::getInstance().handleInterrupt(); }
