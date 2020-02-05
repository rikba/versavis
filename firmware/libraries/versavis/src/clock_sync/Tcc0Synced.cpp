#include "clock_sync/Tcc0Synced.h"

#include "helper.h"

Tcc0Synced::Tcc0Synced() : TccSynced((Tcc *)TCC0) {
  // This is a 32 bit counter.
  top_ = 0xFFFFFFFF;
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC0_IRQn, 0x01);
  NVIC_EnableIRQ(TCC0_IRQn);
}

void TCC0_Handler() { DEBUG_PRINTLN("[Tcc0Synced]: TCC0_Handler."); }
