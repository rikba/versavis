#include "clock_sync/RtcSync.h"
#include "clock_sync/TccSynced.h"

#include "helper.h"

TccSynced::TccSynced(const MfrqPin &mfrq_pin, const ExposurePin &exp_pin,
                     Tcc *tcc)
    : TimerSynced(mfrq_pin), exposure_pin_(exp_pin), tcc_(tcc) {
  setup();
}

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

  DEBUG_PRINTLN("[TccSynced]: Setup EVCTRL to start on RTC overflow.");
  tcc_->EVCTRL.reg |= TCC_EVCTRL_TCEI0 | TCC_EVCTRL_EVACT0_START;

  DEBUG_PRINTLN("[TccSynced]: Enabling event interrupts.");
  tcc_->INTENSET.reg |= TCC_INTENSET_OVF;
  DEBUG_PRINTLN("[TccSynced]: Clearing interrupt flags.");
  tcc_->INTFLAG.reg |= TCC_INTFLAG_OVF;
}

void TccSynced::setupMfrqWaveform() {
  // Setup wavegen.
  DEBUG_PRINTLN("[TccSynced]: Disabling timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINT("[TccSynced]: Prescaling timer by ");
  DEBUG_PRINTLN(kPrescalers[prescaler_]);
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

  // Negate to emit the first pulse at the first tick.
  if (!trigger_state_.invert_) {
    tcc_->DRVCTRL.reg |= TCC_DRVCTRL_INVEN0;
    while (tcc_->SYNCBUSY.bit.ENABLE) {
    }
  }

  DEBUG_PRINT("[TccSynced]: Set FRQ top.");
  updateTopCompare();

  DEBUG_PRINT("[TccSynced]: Set CC3 value to half top: ");
  DEBUG_PRINTLN(top_ / 2);
  while (tcc_->SYNCBUSY.bit.CC3) {
  }
  tcc_->CC[3].reg = top_ / 2;
  while (tcc_->SYNCBUSY.bit.CC3) {
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
void TccSynced::setupMpwmWaveform() { DEBUG_PRINTLN("MPWM not implemented."); }

void TccSynced::updateRate(const uint16_t rate_hz) {
  if (tcc_->WAVE.bit.WAVEGEN == TCC_WAVE_WAVEGEN_MFRQ_Val) {
    updateRateMfrq(rate_hz);
  }
}

void TccSynced::setExposureStateNum(const uint32_t num) {
  exposure_state_.setNum(num);
}

void TccSynced::setupExposure(const bool invert) {
  DEBUG_PRINT("[TccSynced]: Configuring exposure pin ");
  DEBUG_PRINT(exposure_pin_.pin);
  DEBUG_PRINT(" of group ");
  DEBUG_PRINTLN(exposure_pin_.group);

  exposure_state_.invert_ = invert;

  DEBUG_PRINTLN("[TccSynced]: Setup interrupt pin.");
  setupInterruptPin(exposure_pin_.group, exposure_pin_.pin,
                    InterruptLogic::kBoth, false);
  DEBUG_PRINTLN("[TccSynced]: Setup exposure evsys.");
  setupExposureEvsys();

  // Setup interrupt
  DEBUG_PRINTLN("[TccSynced]: Disabling timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  DEBUG_PRINTLN("[TccSynced]: Configure exposure capture and interrupt.");
  tcc_->EVCTRL.reg |= TCC_EVCTRL_MCEI1;
  DEBUG_PRINTLN("[TccSynced]: TCC_CTRLA_CPTEN1.");
  tcc_->CTRLA.reg |= TCC_CTRLA_CPTEN1;
  DEBUG_PRINTLN("[TccSynced]: TCC_INTENSET_MC1.");
  tcc_->INTENSET.reg |= TCC_INTENSET_MC1;
  DEBUG_PRINTLN("[TccSynced]: TCC_INTFLAG_MC1.");
  tcc_->INTFLAG.reg |= TCC_INTFLAG_MC1;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[TccSynced]: Enable timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
}

void TccSynced::setupExternalEvent(const bool invert) {
  DEBUG_PRINT("[TcSynced]: External event not implemented.");
  if (nh_) {
    nh_->logerror("External event not implemented.");
  }
}

uint8_t TccSynced::getEventGeneratorId(const uint8_t pin) const {
  // https://github.com/ethz-asl/versavis_hw/blob/1e71a3843aefbbec8e6261c0855bd7cad7f38f9e/VersaVIS/bootloaders/mzero/Bootloader_D21/src/ASF/sam0/utils/cmsis/samd21/include/instance/evsys.h
  return (pin % 16) + 12;
}

bool TccSynced::getExposurePinValue() const {
  return getPinValue(exposure_pin_.group, exposure_pin_.pin);
}

bool TccSynced::getTimeLastPps(ros::Time *time, uint32_t *num) {
  return pps_state_.getTime(time, num);
}

bool TccSynced::getTimeLastExposure(ros::Time *time, uint32_t *num,
                                    ros::Duration *exp) {
  return exposure_state_.getTime(time, num, exp);
}

void TccSynced::updateTopCompare() {
  // TODO(rikba): Use circular buffer or check register synchronization.
  if (updateFreq()) {
    tcc_->CC[3].reg = top_ / 2;
  }
  tcc_->CC[0].reg = top_ + computeLeapTicks() - 1 + offset_;
  accumulated_offset_ += offset_;
  offset_ = 0;
}

void TccSynced::handleInterrupt() {
  if (tcc_->INTFLAG.bit.OVF) { // Handle overflow.
    tcc_->INTFLAG.reg = TCC_INTFLAG_OVF;
    time_ +=
        RtcSync::getInstance().computeDuration(tcc_->CC[0].reg + 1, prescaler_);
    updateTopCompare();
    syncRtc();
  } else if (tcc_->INTFLAG.bit.MC3) { // Handle half a cycle update.
    tcc_->INTFLAG.reg = TCC_INTFLAG_MC3;
    time_2_ = time_;
    time_2_cc_ = tcc_->CC[3].reg;
  }
  // Handle trigger which comes at the same time as overflow.
  else if (tcc_->INTFLAG.bit.MC0) {
    tcc_->INTFLAG.reg = TCC_INTFLAG_MC0;
    if (getWaveOutPinValue() ^ trigger_state_.invert_) {
      // Capture new trigger pulse.
      trigger_state_.setTime(time_);
    }
  } else if (tcc_->INTFLAG.bit.MC1) { // Capture exposure.
    // TODO(rikba): Find a way to solve half cycle ambiguity for TCC1 and TCC2.
    // https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/225035
    bool missed_start = tcc_->STATUS.bit.CCBV1;
    auto cc = tcc_->CC[1].reg;
    // TODO(rikba): Make sure to not miss any.
    if (missed_start ||
        getExposurePinValue() ^ exposure_state_.invert_) { // Start exposure.
      auto time = cc < time_2_cc_ ? time_ : time_2_;
      time += RtcSync::getInstance().computeDuration(cc, prescaler_);
      exposure_state_.setStart(time);
    } else { // Stop exposure.
      auto time = cc < time_2_cc_ ? time_ : time_2_;
      time += RtcSync::getInstance().computeDuration(cc, prescaler_);
      exposure_state_.setEnd(time);
    }
  } else if (tcc_->INTFLAG.bit.MC2) { // Handle PPS.
    tcc_->INTFLAG.reg = TCC_INTFLAG_MC2;
    auto time = tcc_->CC[2].reg < time_2_cc_ ? time_ : time_2_;
    time += RtcSync::getInstance().computeDuration(tcc_->CC[2].reg, prescaler_);
    pps_state_.setTime(time);
  }
}
