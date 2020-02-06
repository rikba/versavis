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

  DEBUG_PRINTLN("[TcSynced]: Enabling event interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC0 | TC_INTENSET_OVF | TC_INTENSET_ERR;
  DEBUG_PRINTLN("[TcSynced]: Clearing interrupt flags.");
  tc_->INTFLAG.reg |= TC_INTENSET_MC0 | TC_INTFLAG_OVF | TC_INTENSET_ERR;
}

void TcSynced::setupPwm(uint16_t rate_hz, uint32_t pulse_us, bool invert) {
  // Compute prescaler
  prescaler_ = findMinPrescaler(rate_hz, RTC_FREQ, top_);
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

  DEBUG_PRINTLN("[TcSynced]: Activate match PWM.");
  tc_->CTRLA.reg |= TC_CTRLA_WAVEGEN_MPWM;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Make channel 0 and 1 compare register.");
  tc_->CTRLC.reg &= ~TC_CTRLC_CPTEN0;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLC.reg &= ~TC_CTRLC_CPTEN1;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  if (invert) {
    DEBUG_PRINTLN("[TcSynced]: Invert PWM.");
    tc_->CTRLC.reg &= ~TC_CTRLC_INVEN1;
    while (tc_->STATUS.bit.SYNCBUSY) {
    }
  }

  uint32_t duty_cycle = 0;
  RtcSync::getInstance().computePwm(rate_hz, pulse_us, kPrescalers[prescaler_],
                                    &top_, &duty_cycle);
  DEBUG_PRINT("[TcSynced]: Setup pwm top: ");
  DEBUG_PRINT(top_);
  DEBUG_PRINT(" duty cycle: ");
  DEBUG_PRINTLN(duty_cycle);
  tc_->CC[0].reg = top_;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CC[1].reg = duty_cycle;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  // Setup output pin.
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.
  DEBUG_PRINTLN("[TcSynced]: Configuring port PA19 TC3/WO[1] PWM pin.");
  PORT->Group[PORTA].PMUX[19 >> 1].reg |= PORT_PMUX_PMUXO_E; // TC3/WO[1]
  PORT->Group[PORTA].PINCFG[19].reg |= PORT_PINCFG_PMUXEN;   // Multiplexation
}

void TcSynced::begin() const {
  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
}

void TcSynced::handleInterrupt() {
  DEBUG_PRINTLN("handleInterrupt");
  DEBUG_PRINTLN(tc_->INTFLAG.bit.MC0);
  DEBUG_PRINTLN(tc_->INTFLAG.bit.MC1);
  DEBUG_PRINTLN(tc_->INTFLAG.bit.OVF);
  syncRtc();

  if (tc_->INTFLAG.bit.MC0 && tc_->INTFLAG.bit.OVF) {
    DEBUG_PRINTLN("[TcSynced]: pwmPulse.");
    pwmPulse();
  } else if (tc_->INTFLAG.bit.OVF) {
    DEBUG_PRINTLN("[TcSynced]: overflow.");
    overflow();
  }

  if (tc_->INTFLAG.bit.ERR) {
    error("INTFLAG_ERR (TcSynced.cpp): match or capture value already set",
          201);
  }

  // Clear flags.
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.MC0;
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.MC1;
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.OVF;
}
