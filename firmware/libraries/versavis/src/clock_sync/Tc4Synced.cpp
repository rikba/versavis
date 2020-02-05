#include "clock_sync/Tc4Synced.h"

Tc4Synced::Tc4Synced() : TcSynced((TcCount16 *)TC4) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC4_IRQn, 0x01);
  NVIC_EnableIRQ(TC4_IRQn);
}

void TC4_Handler() { Tc4Synced::getInstance().handleInterrupt(); }
