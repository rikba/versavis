#include "clock_sync/TcSynced.h"

#include "clock_sync/RtcSync.h"

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

  DEBUG_PRINTLN("[TcSynced]: Capture channel 1.");
  tc_->CTRLC.reg |= TC_CTRLC_CPTEN1;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Enabling event interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC1 | TC_INTENSET_OVF;
  DEBUG_PRINTLN("[TcSynced]: Clearing interrupt flags.");
  tc_->INTFLAG.reg |= TC_INTENSET_MC1 | TC_INTFLAG_OVF;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
}

void TcSynced::setupMfrq(uint16_t rate_hz, bool invert) {
  invert_trigger_ = invert;
  // Compute prescaler
  prescaler_ = findMinPrescalerFrq(rate_hz, RTC_FREQ, top_);
  DEBUG_PRINT("[TcSynced]: Prescaling timer by ");
  DEBUG_PRINTLN(kPrescalers[prescaler_]);

  // Setup wavegen.
  DEBUG_PRINTLN("[TcSynced]: Disabling timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Set prescaler.");
  tc_->CTRLA.reg |= TC_CTRLA_PRESCALER(prescaler_);
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Activate MFRQ.");
  tc_->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Make channel 0 compare register.");
  tc_->CTRLC.reg &= ~TC_CTRLC_CPTEN0;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  // Invert the inversion to start with a high pin at t = 0.
  if (!invert) {
    tc_->CTRLC.reg |= TC_CTRLC_INVEN0;
    while (tc_->STATUS.bit.SYNCBUSY) {
    }
  }

  RtcSync::getInstance().computeFrq(rate_hz, kPrescalers[prescaler_], &top_);
  DEBUG_PRINT("[TcSynced]: Set FRQ top: ");
  DEBUG_PRINTLN(top_);
  tc_->CC[0].reg = top_;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  // DEBUG_PRINTLN("[TcSynced]: Set counter top to trigger immediately on
  // start."); tc_->COUNT.reg = TC_COUNT16_COUNT_COUNT(top_);

  DEBUG_PRINTLN("[TcSynced]: Enabling MFRQ interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC0;
  tc_->INTFLAG.reg |= TC_INTENSET_MC0;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  // Setup output pin.
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.
  DEBUG_PRINTLN("[TcSynced]: Configuring port PA14 TC3/WO[0] PWM pin.");
  PORT->Group[PORTA].PMUX[14 >> 1].reg |= PORT_PMUX_PMUXE_E; // TC3/WO[0]
  PORT->Group[PORTA].PINCFG[14].reg |= PORT_PINCFG_PMUXEN;   // Multiplexation
}

void TcSynced::handleInterrupt() {
  DEBUG_PRINTLN("handleInterrupt");
  DEBUG_PRINTLN(tc_->INTFLAG.bit.MC0);
  DEBUG_PRINTLN(tc_->INTFLAG.bit.MC1);
  DEBUG_PRINTLN(tc_->INTFLAG.bit.OVF);
  if (PORT->Group[PORTA].IN.reg & (1 << 14)) {
    DEBUG_PRINTLN("HIGH");
  } else {
    DEBUG_PRINTLN("LOW");
  }

  bool pin = PORT->Group[PORTA].IN.reg & (1 << 14);

  if (tc_->INTFLAG.bit.MC0 && (pin ^ invert_trigger_)) {
    DEBUG_PRINTLN("trigger");
    trigger();
  }

  if (tc_->INTFLAG.bit.MC1) {
    DEBUG_PRINTLN("syncRtc");
    syncRtc();
  } else if (tc_->INTFLAG.bit.OVF) {
    DEBUG_PRINTLN("overflow");
    overflow();
  }

  // Clear flags.
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.MC0;
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.MC1;
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.OVF;
}
