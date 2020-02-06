#include "clock_sync/Tcc0Synced.h"

#include "helper.h"

Tcc0Synced::Tcc0Synced() : TccSynced((Tcc *)TCC0) {
  // This is a 32 bit counter.
  top_ = 0xFFFFFFFF;
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC0_IRQn, 0x01);
  NVIC_EnableIRQ(TCC0_IRQn);
}

void Tcc0Synced::setupOutPin() {
  DEBUG_PRINTLN(
      "[Tcc0Synced]: Configuring port PA04 TCC0/WO[0] wave output pin.");
  PORT->Group[PORTA].PMUX[4 >> 1].reg |= PORT_PMUX_PMUXE_E;
  PORT->Group[PORTA].PINCFG[4].reg |= PORT_PINCFG_PMUXEN;
}

void TCC0_Handler() { DEBUG_PRINTLN("[Tcc0Synced]: TCC0_Handler."); }
