#include "clock_sync/Tc5Synced.h"

#include "helper.h"

Tc5Synced::Tc5Synced() : TcSynced((TcCount16 *)TC5) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC5_IRQn, 0x01);
  NVIC_EnableIRQ(TC5_IRQn);
}

void Tc5Synced::setupOutPin() const {
  DEBUG_PRINTLN("[Tc5Synced]: Wave output pin TC5/WO[0] not configured!");
}

bool Tc5Synced::getOutPinValue() const {
  DEBUG_PRINTLN("[Tc5Synced]: Wave output pin TC5/WO[0] not configured!");
  return invert_trigger_;
}

void TC5_Handler() { Tc5Synced::getInstance().handleInterrupt(); }
