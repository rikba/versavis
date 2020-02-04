#include "sensors/Tcc2Synced.h"

#include "helper.h"

Tcc2Synced::Tcc2Synced() : TccSynced((Tcc *)TCC2) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC2_IRQn, 0x01);
  NVIC_EnableIRQ(TCC2_IRQn);
}

void TCC2_Handler() { DEBUG_PRINTLN("[Tcc2Synced]: TCC2_Handler."); }
