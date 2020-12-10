#include "clock_sync/TimerSynced.h"

#include "clock_sync/RtcSync.h"
#include "helper.h"

TimerSynced::TimerSynced(const MfrqPin &mfrq_pin) : mfrq_pin_(mfrq_pin) {
  RtcSync::getInstance(); // Make sure RTC singleton exists.
}

void TimerSynced::updateRateMfrq(const uint16_t rate_hz) {
  new_freq_ = 2 * rate_hz;
}

void TimerSynced::updateRateMpwm(const uint16_t rate_hz) {
  new_freq_ = rate_hz;
}

void TimerSynced::offsetTrigger(const double sec) {
  double offset = (RTC_FREQ / kPrescalers[prescaler_]) * sec;
  offset = offset + 0.5 - (offset < 0);
  offset_ = static_cast<int32_t>(offset);
}

bool TimerSynced::updateFreq() {
  bool update = (new_freq_ != freq_);
  if (update) {
    // Only update at change of second.
    auto offset_time = time_;
    offset_time -=
        RtcSync::getInstance().computeDuration(accumulated_offset_, prescaler_);
    update &= !offset_time.nsec;
    if (update) {
      setClosestRate(new_freq_);
    }
  }
  return update;
}

void TimerSynced::setClosestRate(const uint16_t freq) {
  if (freq_ != freq && r_ == 0) {
    // Find closest possible rate.
    uint16_t min_freq = RTC_FREQ / (kPrescalers[prescaler_] * top_max_);
    min_freq += ((static_cast<uint32_t>(RTC_FREQ) %
                  (kPrescalers[prescaler_] * top_max_)) != 0);
    freq_ = freq >= min_freq ? freq : min_freq;
    new_freq_ = freq_;

    wrap_around_ = kPrescalers[prescaler_] * freq_;
    top_ = RTC_FREQ / wrap_around_;
    mod_ = static_cast<uint32_t>(RTC_FREQ) % wrap_around_;
    // if (nh_) {
    //   char buffer[150];
    //   sprintf(buffer,
    //           "Changed rate. Desired rate: %d Actual rate: %d Top compare: %u
    //           " "Prescaler: %d Wrap around: %d", freq, freq_, top_,
    //           kPrescalers[prescaler_], wrap_around_);
    //   nh_->loginfo(buffer);
    // }
  } else if (r_ != 0) {
    nh_->logerror("Cannot update rate while remainder r is not zero.");
  }
}

void TimerSynced::setupMfrq(const uint16_t rate_hz) {
  // Set parameters.
  prescaler_ = RtcSync::getInstance().findMinPrescalerFrq(rate_hz, top_max_);
  setClosestRate(2 * rate_hz);

  // Setup timer specific match frequency configuration.
  setupMfrqWaveform();

  // Setup output pin.
  setupWaveOutPin();
}

void TimerSynced::setupMpwm(const uint16_t rate_hz, const uint16_t pulse_us) {
  // Set parameters.
  prescaler_ = RtcSync::getInstance().findMinPrescalerPwm(rate_hz, top_max_);
  setClosestRate(rate_hz);

  uint32_t ticks = (RTC_FREQ / 1e6) * pulse_us;
  pulse_ticks_ = ticks / kPrescalers[prescaler_];
  pulse_ticks_ += ((ticks % kPrescalers[prescaler_]) != 0); // Ceil

  // Setup timer specific match frequency configuration.
  setupMpwmWaveform();

  // Setup output pin.
  setupWaveOutPin();
}

void TimerSynced::setLatestMeasurementNum(const uint32_t num) {
  measurement_state_.setNum(num);
}

uint16_t TimerSynced::computeLeapTicks() {
  r_ += mod_;
  uint16_t leap_ticks = r_ / wrap_around_;
  r_ %= wrap_around_;
  return leap_ticks;
}

void TimerSynced::setupWaveOutPin() const {
  DEBUG_PRINT("[TimerSynced]: Configuring wave output pin ");
  DEBUG_PRINT(mfrq_pin_.pin);
  DEBUG_PRINT(" of group ");
  DEBUG_PRINTLN(mfrq_pin_.group);
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.
  if (mfrq_pin_.pin % 2) {
    PORT->Group[mfrq_pin_.group].PMUX[mfrq_pin_.pin >> 1].reg |=
        PORT_PMUX_PMUXO_E;
  } else {
    PORT->Group[mfrq_pin_.group].PMUX[mfrq_pin_.pin >> 1].reg |=
        PORT_PMUX_PMUXE_E;
  }
  PORT->Group[mfrq_pin_.group].PINCFG[mfrq_pin_.pin].reg |= PORT_PINCFG_PMUXEN;

  if (mfrq_pin_.drvstr) {
    DEBUG_PRINTLN("[TimerSynced]: Setting pin strong.");
    PORT->Group[mfrq_pin_.group].PINCFG[mfrq_pin_.pin].reg |=
        PORT_PINCFG_DRVSTR;
  }
}

void TimerSynced::setupInterruptPin(const uint8_t port_group, const uint8_t pin,
                                    const InterruptLogic &logic,
                                    const bool enable_interrupt) const {

  // PORT configurations.
  REG_PM_APBBMASK |= PM_APBBMASK_PORT;

  DEBUG_PRINTLN("[TimerSynced]: Configure pin input.");
  PORT->Group[port_group].DIRCLR.reg = PORT_DIRCLR_DIRCLR(1 << pin);

  DEBUG_PRINTLN("[TimerSynced]: Connect interrupt pin with PMUX A.");
  if (pin % 2) {
    PORT->Group[port_group].PMUX[pin >> 1].reg |= PORT_PMUX_PMUXO_A; // Odd
  } else {
    PORT->Group[port_group].PMUX[pin >> 1].reg |= PORT_PMUX_PMUXE_A; // Even
  }
  PORT->Group[port_group].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[port_group].PINCFG[pin].reg |= PORT_PINCFG_INEN;

  // Activate EIC edge detection.
  if (logic == InterruptLogic::kRise || logic == InterruptLogic::kFall ||
      logic == InterruptLogic::kBoth) {
    DEBUG_PRINTLN(
        "[TimerSynced]: Enabling generic clock 4 for edge detection.");
    GCLK->CLKCTRL.reg =
        GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_EIC;
    while (GCLK->STATUS.bit.SYNCBUSY) {
    } // Wait for synchronization
  }

  // EIC configurations.
  REG_PM_APBAMASK |= PM_APBAMASK_EIC;
  DEBUG_PRINTLN("[TimerSynced]: Disable EIC.");
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
  EIC->CTRL.reg &= ~EIC_CTRL_ENABLE;
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[TimerSynced]: Configure EXTINTEO.");
  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO(1 << (pin % 16));

  uint8_t config_id = pin < 16 ? 1 : 2;
  config_id = pin < 8 ? 0 : config_id;
  DEBUG_PRINT("[TimerSynced]: Configure SENSE with CONFIG");
  DEBUG_PRINTLN(config_id);
  uint8_t id = pin % 8;
  uint32_t logic_val = static_cast<uint32_t>(logic);
  switch (id) {
  case 0:
    EIC->CONFIG[config_id].bit.SENSE0 = logic_val;
    break;
  case 1:
    EIC->CONFIG[config_id].bit.SENSE1 = logic_val;
    break;
  case 2:
    EIC->CONFIG[config_id].bit.SENSE2 = logic_val;
    break;
  case 3:
    EIC->CONFIG[config_id].bit.SENSE3 = logic_val;
    break;
  case 4:
    EIC->CONFIG[config_id].bit.SENSE4 = logic_val;
    break;
  case 5:
    EIC->CONFIG[config_id].bit.SENSE5 = logic_val;
    break;
  case 6:
    EIC->CONFIG[config_id].bit.SENSE6 = logic_val;
    break;
  case 7:
    EIC->CONFIG[config_id].bit.SENSE7 = logic_val;
    break;
  }

  if (enable_interrupt) {
    DEBUG_PRINTLN("[TimerSynced]: Enable interrupt handler.");
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT(1 << (pin % 16));
    EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(1 << (pin % 16));
  }

  DEBUG_PRINTLN("[TimerSynced]: Enable EIC.");
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
  EIC->CTRL.reg |= EIC_CTRL_ENABLE;
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  if (enable_interrupt) {
    NVIC_SetPriority(EIC_IRQn, 2);
    NVIC_EnableIRQ(EIC_IRQn);
  }
}

bool TimerSynced::getPinValue(const uint8_t group, const uint8_t pin) const {
  return PORT->Group[group].IN.reg & (1 << pin);
}

bool TimerSynced::getWaveOutPinValue() const {
  return getPinValue(mfrq_pin_.group, mfrq_pin_.pin);
}

bool TimerSynced::getMeasurement(Measurement *meas) {
  return measurement_state_.getMeasurement(meas);
}

void TimerSynced::handleEic() {
  if (EIC->INTFLAG.vec.EXTINT & (1 << (dr_pin_ % 16))) {
    measurement_state_.setDataReady();
    EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(1 << (dr_pin_ % 16));
  }
}

void TimerSynced::syncRtc() {
  // Do not sync exactly at second wrap around.
  if (time_.nsec > 2e8 && time_.nsec < 8e8) {
    if (RtcSync::getInstance().getSec() != time_.sec) {
      time_.sec = RtcSync::getInstance().getSec();
    }
  }
}
