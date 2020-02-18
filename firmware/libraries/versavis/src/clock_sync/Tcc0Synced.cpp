#include "clock_sync/Tcc0Synced.h"

// Setup MFRQ pin.
#define TCC0_MFRQ_GROUP PORTA
#define TCC0_MFRQ_PIN 4
#define TCC0_MFRQ_DRVSTR 1

Tcc0Synced::Tcc0Synced()
    : TccSynced(MfrqPin{.group = TCC0_MFRQ_GROUP,
                        .pin = TCC0_MFRQ_PIN,
                        .drvstr = TCC0_MFRQ_DRVSTR},
                (Tcc *)TCC0) {
  // This is a 32 bit counter.
  top_ = 0xFFFFFFFF;
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC0_IRQn, 0x01);
  NVIC_EnableIRQ(TCC0_IRQn);
}

void TCC0_Handler() { Tcc0Synced::getInstance().handleInterrupt(); }
