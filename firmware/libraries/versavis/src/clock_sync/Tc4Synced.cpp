#include "clock_sync/Tc4Synced.h"

#include "helper.h"

Tc4Synced::Tc4Synced() : TcSynced((TcCount16 *)TC4) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC4_IRQn, 0x01);
  NVIC_EnableIRQ(TC4_IRQn);
}

void Tc4Synced::setupOutPin() const {
  DEBUG_PRINTLN("[Tc4Synced]: Wave output pin TC4/WO[0] not configured!");
}

bool Tc4Synced::getOutPinValue() const {
  DEBUG_PRINTLN("[Tc4Synced]: Wave output pin TC4/WO[0] not configured!");
  return invert_trigger_;
}

void TC4_Handler() { Tc4Synced::getInstance().handleInterrupt(); }
