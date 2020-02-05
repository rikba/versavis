#include "sensors/Tc3Synced.h"

Tc3Synced::Tc3Synced() : TcSynced((TcCount16 *)TC3) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC3_IRQn, 0x01);
  NVIC_EnableIRQ(TC3_IRQn);
}

void TC3_Handler() { Tc3Synced::getInstance().handleInterrupt(); }
