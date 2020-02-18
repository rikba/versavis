#include "clock_sync/TccSynced.h"

#include "helper.h"

TccSynced::TccSynced(Tcc *tcc) : tcc_(tcc) { setup(); }

void TccSynced::setup() const {
  if (!tcc_) {
    error("NO_TIMER (TccSynced.cpp): timer does not exist.", 201);
  }

  DEBUG_PRINTLN("[TccSynced]: Disabling timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[TccSynced]: Setup EVCTRL to retrigger on RTC overflow.");
  tcc_->EVCTRL.reg |= TCC_EVCTRL_TCEI0 | TCC_EVCTRL_EVACT0_RETRIGGER;

  DEBUG_PRINTLN("[TccSynced]: Enabling event interrupts.");
  tcc_->INTENSET.reg |= TCC_INTENSET_TRG | TCC_INTENSET_OVF;
  DEBUG_PRINTLN("[TccSynced]: Clearing interrupt flags.");
  tcc_->INTFLAG.reg |= TCC_INTFLAG_TRG | TCC_INTFLAG_OVF;

  DEBUG_PRINTLN("[TccSynced]: Enable timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[TccSynced]: Setup done..");
}

void TccSynced::setupPwm(uint16_t rate_hz, uint32_t pulse_us, bool invert) {
  DEBUG_PRINTLN("[TcSynced]: IMPLEMENT!!");
}

void TccSynced::handleInterrupt() {
  if (tcc_->INTFLAG.bit.TRG) {
    syncRtc();
  }

  // Clear flags.
  tcc_->INTFLAG.reg |= tcc_->INTFLAG.bit.TRG;
  tcc_->INTFLAG.reg |= tcc_->INTFLAG.bit.OVF;
}
