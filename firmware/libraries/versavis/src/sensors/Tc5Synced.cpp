#include "sensors/Tc5Synced.h"

#include "helper.h"

Tc5Synced::Tc5Synced() : TcSynced((TcCount16 *)TC5) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC5_IRQn, 0x01);
  NVIC_EnableIRQ(TC5_IRQn);
}

void TC5_Handler() {
  DEBUG_PRINTLN("[Tc5Synced]: TC5_Handler.");
  Tc5Synced::getInstance().handleInterrupt();
}
