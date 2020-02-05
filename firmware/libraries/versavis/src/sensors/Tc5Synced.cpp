#include "sensors/Tc5Synced.h"

Tc5Synced::Tc5Synced() : TcSynced((TcCount16 *)TC5) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC5_IRQn, 0x01);
  NVIC_EnableIRQ(TC5_IRQn);
}

void TC5_Handler() { Tc5Synced::getInstance().handleInterrupt(); }
