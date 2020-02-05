#include "clock_sync/TcSynced.h"

#include "helper.h"
#include "versavis_configuration.h"

TcSynced::TcSynced(TcCount16 *tc) : tc_(tc) {}

void TcSynced::setup() const {
  DEBUG_PRINTLN("[TcSynced]: setupTimer.");
  if (!tc_) {
    error("NO_TIMER (TcSynced.cpp): timer does not exist.", 201);
  }

  DEBUG_PRINTLN("[TcSynced]: Disabling timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Setup EVCTRL to retrigger on RTC overflow.");
  tc_->EVCTRL.reg |= TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_RETRIGGER;

  DEBUG_PRINTLN("[TcSynced]: Setup CTRLC to capture event input.");
  tc_->CTRLC.reg |= TC_CTRLC_CPTEN0;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Enabling event interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC0 | TC_INTENSET_OVF;
  DEBUG_PRINTLN("[TcSynced]: Clearing interrupt flags.");
  tc_->INTFLAG.reg |= TC_INTFLAG_MC0 | TC_INTFLAG_OVF;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
}

void TcSynced::handleRetrigger() {
  if (tc_->INTFLAG.bit.MC0) {
    retrigger();
    tc_->INTFLAG.reg |= TC_INTFLAG_MC0;
  }
}

void TcSynced::handleOverflow() {
  if (tc_->INTFLAG.bit.OVF) {
    overflow();
    tc_->INTFLAG.reg |= TC_INTFLAG_OVF;
  }
}
