#include "clock_sync/Tc3Synced.h"

// Setup MFRQ pin.
#define TC3_MFRQ_GROUP PORTA
#define TC3_MFRQ_PIN 14
#define TC3_MFRQ_DRVSTR 0

Tc3Synced::Tc3Synced()
    : TcSynced(MfrqPin{.group = TC3_MFRQ_GROUP,
                       .pin = TC3_MFRQ_PIN,
                       .drvstr = TC3_MFRQ_DRVSTR},
               (TcCount16 *)TC3) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC3_IRQn, 0x01);
  NVIC_EnableIRQ(TC3_IRQn);
}

void TC3_Handler() { Tc3Synced::getInstance().handleInterrupt(); }
