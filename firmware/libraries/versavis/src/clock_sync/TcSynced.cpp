#include "clock_sync/TcSynced.h"

#include "clock_sync/RtcSync.h"

#include "helper.h"
#include "versavis_configuration.h"

TcSynced::TcSynced(const MfrqPin &mfrq_pin, TcCount16 *tc)
    : TimerSynced(mfrq_pin), tc_(tc) {
  setup();
}

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

  DEBUG_PRINTLN("[TcSynced]: Setup EVCTRL to start on RTC overflow.");
  tc_->EVCTRL.reg |= TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_START;

  DEBUG_PRINTLN("[TcSynced]: Enabling event interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_OVF;
  DEBUG_PRINTLN("[TcSynced]: Clearing interrupt flags.");
  tc_->INTFLAG.reg |= TC_INTFLAG_OVF;
}

void TcSynced::setupMpwmWaveform() {
  // Setup wavegen.
  DEBUG_PRINTLN("[TcSynced]: Disabling timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINT("[TcSynced]: Prescaling timer by ");
  DEBUG_PRINTLN(kPrescalers[prescaler_]);
  tc_->CTRLA.reg |= TC_CTRLA_PRESCALER(prescaler_);
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Activate MPWM.");
  tc_->CTRLA.reg |= TC_CTRLA_WAVEGEN_MPWM;

  DEBUG_PRINTLN("[TcSynced]: Make channel 0 compare register.");
  tc_->CTRLC.reg &= ~TC_CTRLC_CPTEN0;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Make channel 1 compare register.");
  tc_->CTRLC.reg &= ~TC_CTRLC_CPTEN1;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  // Invert pulse if desired.
  if (trigger_state_.invert_) {
    tc_->CTRLC.reg |= TC_CTRLC_INVEN1;
    while (tc_->STATUS.bit.SYNCBUSY) {
    }
  }

  DEBUG_PRINT("[TcSynced]: Set FRQ top.");
  updateTopCompare();

  DEBUG_PRINTLN("[TcSynced]: Setup pulse width WO[1].");
  tc_->CC[1].reg = pulse_ticks_;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Enabling MC1 interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC1;
  tc_->INTFLAG.reg |= TC_INTFLAG_MC1;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
}

void TcSynced::updateRate(const uint16_t rate_hz) {
  if (tc_->CTRLA.bit.WAVEGEN == TC_CTRLA_WAVEGEN_MFRQ_Val) {
    updateRateMfrq(rate_hz);
  } else if (tc_->CTRLA.bit.WAVEGEN == TC_CTRLA_WAVEGEN_MPWM_Val) {
    updateRateMpwm(rate_hz);
  }
}

void TcSynced::setupMfrqWaveform() {
  // Setup wavegen.
  DEBUG_PRINTLN("[TcSynced]: Disabling timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINT("[TcSynced]: Prescaling timer by ");
  DEBUG_PRINTLN(kPrescalers[prescaler_]);
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

  // Negate to emit the first pulse at the first tick.
  if (!trigger_state_.invert_) {
    tc_->CTRLC.reg |= TC_CTRLC_INVEN0;
    while (tc_->STATUS.bit.SYNCBUSY) {
    }
  }

  DEBUG_PRINT("[TcSynced]: Set FRQ top.");
  updateTopCompare();

  DEBUG_PRINTLN("[TcSynced]: Enabling MFRQ interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC0;
  tc_->INTFLAG.reg |= TC_INTFLAG_MC0;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
}

void TcSynced::setupDataReady(const uint8_t port_group, const uint8_t pin,
                              const InterruptLogic &logic) {
  // Store parameters.
  dr_port_group_ = port_group;
  dr_pin_ = pin;

  setupInterruptPin(port_group, pin, logic, true);
}

void TcSynced::updateTopCompare() {
  updateFreq();
  tc_->CC[0].reg = top_ + computeLeapTicks() - 1 + offset_;
  accumulated_offset_ += offset_;
  offset_ = 0;
}

void TcSynced::handleInterrupt() {

  // Handle overflow.
  if (tc_->INTFLAG.bit.OVF) {
    tc_->INTFLAG.reg = TC_INTFLAG_OVF;
    time_ +=
        RtcSync::getInstance().computeDuration(tc_->CC[0].reg + 1, prescaler_);
    updateTopCompare();
    syncRtc();
  }
  // Handle trigger which comes at the same time as overflow.
  else if (tc_->INTFLAG.bit.MC0) {
    tc_->INTFLAG.reg = TC_INTFLAG_MC0;
    if (getWaveOutPinValue() ^ trigger_state_.invert_) {
      // Set new trigger timestamp.
      trigger_state_.setTime(time_);
    }
  }
  // Handle MPWM end of pulse to stamp new message.
  else if (tc_->INTFLAG.bit.MC1) {
    tc_->INTFLAG.reg = TC_INTFLAG_MC1;
    trigger_state_.setTime(time_);
  }
}
