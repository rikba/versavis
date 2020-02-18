#include "clock_sync/Tcc2Synced.h"

// Setup MFRQ pin.
// TODO(rikba)
#define TCC2_MFRQ_GROUP 255
#define TCC2_MFRQ_PIN 255
#define TCC2_MFRQ_DRVSTR 0

Tcc2Synced::Tcc2Synced()
    : TccSynced(MfrqPin{.group = TCC2_MFRQ_GROUP,
                        .pin = TCC2_MFRQ_PIN,
                        .drvstr = TCC2_MFRQ_DRVSTR},
                (Tcc *)TCC2) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC2_IRQn, 0x01);
  NVIC_EnableIRQ(TCC2_IRQn);
}

void TCC2_Handler() { Tcc2Synced::getInstance().handleInterrupt(); }
