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
}

void TccSynced::setupMfrqWaveform() const {
  // Setup wavegen.
  DEBUG_PRINTLN("[TccSynced]: Disabling timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINT("[TccSynced]: Prescaling timer by ");
  DEBUG_PRINTLN(prescaler_);
  tcc_->CTRLA.reg |= TCC_CTRLA_PRESCALER(prescaler_);
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[TccSynced]: Activate MFRQ.");
  tcc_->WAVE.reg |= TCC_WAVE_WAVEGEN_MFRQ;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[TccSynced]: Make CC0 compare register.");
  tcc_->CTRLA.reg &= ~TCC_CTRLA_CPTEN0;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  if (invert_trigger_) {
    tcc_->DRVCTRL.reg |= TCC_DRVCTRL_INVEN0;
    while (tcc_->SYNCBUSY.bit.ENABLE) {
    }
  }

  DEBUG_PRINT("[TccSynced]: Set FRQ top: ");
  DEBUG_PRINTLN(top_);
  while (tcc_->SYNCBUSY.bit.CC0) {
  }
  tcc_->CC[0].reg = top_;
  while (tcc_->SYNCBUSY.bit.CC0) {
  }

  DEBUG_PRINTLN("[TccSynced]: Enabling MFRQ interrupts.");
  tcc_->INTENSET.reg |= TCC_INTENSET_MC0;
  tcc_->INTFLAG.reg |= TCC_INTFLAG_MC0;

  DEBUG_PRINTLN("[TccSynced]: Enable timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
}

void TccSynced::handleInterrupt() {
  if (tcc_->INTFLAG.bit.MC0 && (getOutPinValue() ^ invert_trigger_)) {
    DEBUG_PRINTLN("[TcSynced]: trigger");
    trigger();
  }
  if (tcc_->INTFLAG.bit.TRG) {
    DEBUG_PRINTLN("[TcSynced]: syncRtc");
    syncRtc();
  }

  // Clear flags.
  tcc_->INTFLAG.reg |= tcc_->INTFLAG.bit.MC0;
  tcc_->INTFLAG.reg |= tcc_->INTFLAG.bit.TRG;
  tcc_->INTFLAG.reg |= tcc_->INTFLAG.bit.OVF;
}
