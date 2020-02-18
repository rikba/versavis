#include "clock_sync/Tcc2Synced.h"

#include "helper.h"

Tcc2Synced::Tcc2Synced() : TccSynced((Tcc *)TCC2) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC2_IRQn, 0x01);
  NVIC_EnableIRQ(TCC2_IRQn);
}

void Tcc2Synced::setupOutPin() {
  DEBUG_PRINTLN("[Tcc0Synced]: Wave output pin TCC2/WO[0] not configured!");
}

bool Tcc2Synced::getOutPinValue() const {
  DEBUG_PRINTLN("[Tcc0Synced]: Wave output pin TCC2/WO[0] not configured!");
  return invert_trigger_;
}

void TCC2_Handler() { Tcc2Synced::getInstance().handleInterrupt(); }
