#include "clock_sync/TccSynced.h"

#include "helper.h"

TccSynced::TccSynced(Tcc *tcc) : tcc_(tcc) {}

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

  // DEBUG_PRINTLN("[TccSynced]: Setup CTRLa to capture event input.");
  // tc_->CTRLC.reg |= TCC_CTRLA_CPTEN0;

  DEBUG_PRINTLN("[TccSynced]: Setup EVCTRL to retrigger on RTC overflow.");
  tcc_->EVCTRL.reg |= TCC_EVCTRL_TCEI1 | TCC_EVCTRL_EVACT1_RETRIGGER;

  DEBUG_PRINTLN("[TccSynced]: Enable timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
}

void TccSynced::setupPwm(uint16_t rate_hz, uint32_t pulse_us, bool invert) {
  DEBUG_PRINTLN("[TcSynced]: IMPLEMENT!!");
}

void TccSynced::handleInterrupt() { DEBUG_PRINTLN("[TccSynced]: IMPLEMENT!!"); }
