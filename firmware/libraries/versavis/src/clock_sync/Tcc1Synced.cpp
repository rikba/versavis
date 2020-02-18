#include "clock_sync/Tcc1Synced.h"

// Setup MFRQ pin.
#define TCC1_MFRQ_GROUP PORTA
#define TCC1_MFRQ_PIN 7
#define TCC1_MFRQ_DRVSTR 0

Tcc1Synced::Tcc1Synced()
    : TccSynced(MfrqPin{.group = TCC1_MFRQ_GROUP,
                        .pin = TCC1_MFRQ_PIN,
                        .drvstr = TCC1_MFRQ_DRVSTR},
                (Tcc *)TCC1) {
  // This is a 32 bit counter.
  top_ = 0xFFFFFFFF;
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC1_IRQn, 0x01);
  NVIC_EnableIRQ(TCC1_IRQn);
}

void TCC1_Handler() { Tcc1Synced::getInstance().handleInterrupt(); }
