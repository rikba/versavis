#include "clock_sync/Tc5Synced.h"

// Setup MFRQ pin.
// TODO(rikba)
#define TC5_MFRQ_GROUP 255
#define TC5_MFRQ_PIN 255
#define TC5_MFRQ_DRVSTR 0

Tc5Synced::Tc5Synced()
    : TcSynced(MfrqPin{.group = TC5_MFRQ_GROUP,
                       .pin = TC5_MFRQ_PIN,
                       .drvstr = TC5_MFRQ_DRVSTR},
               (TcCount16 *)TC5) {
  // Enable interrupts. Highest priority to immediately update time stamps.
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);
}

void TC5_Handler() { Tc5Synced::getInstance().handleInterrupt(); }
