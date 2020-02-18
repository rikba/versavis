#include "clock_sync/Tcc1Synced.h"

#include "helper.h"

Tcc1Synced::Tcc1Synced() : TccSynced((Tcc *)TCC1) {
  // This is a 32 bit counter.
  top_ = 0xFFFFFFFF;
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC1_IRQn, 0x01);
  NVIC_EnableIRQ(TCC1_IRQn);
}

void Tcc1Synced::setupOutPin() const {
  DEBUG_PRINTLN(
      "[Tcc1Synced]: Configuring port PA07 TCC1/WO[1] wave output pin.");
  PORT->Group[PORTA].PMUX[7 >> 1].reg |= PORT_PMUX_PMUXO_E;
  PORT->Group[PORTA].PINCFG[7].reg |= PORT_PINCFG_PMUXEN;
}

bool Tcc1Synced::getOutPinValue() const {
  return PORT->Group[PORTA].IN.reg & (1 << 7);
}

void TCC1_Handler() { Tcc1Synced::getInstance().handleInterrupt(); }
