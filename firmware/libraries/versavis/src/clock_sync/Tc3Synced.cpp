#include "clock_sync/Tc3Synced.h"

#include "helper.h"

Tc3Synced::Tc3Synced() : TcSynced((TcCount16 *)TC3) {
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TC3_IRQn, 0x01);
  NVIC_EnableIRQ(TC3_IRQn);
}

void Tc3Synced::setupOutPin() {
  DEBUG_PRINTLN(
      "[Tc3Synced]: Configuring port PA14 TC3/WO[0] wave output pin.");
  PORT->Group[PORTA].PMUX[14 >> 1].reg |= PORT_PMUX_PMUXE_E;
  PORT->Group[PORTA].PINCFG[14].reg |= PORT_PINCFG_PMUXEN;
}

bool Tc3Synced::getOutPinValue() const {
  return PORT->Group[PORTA].IN.reg & (1 << 14);
}

void TC3_Handler() { Tc3Synced::getInstance().handleInterrupt(); }
