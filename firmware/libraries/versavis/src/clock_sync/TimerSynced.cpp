#include "clock_sync/TimerSynced.h"

#include "clock_sync/RtcSync.h"
#include "helper.h"

TimerSynced::TimerSynced(const MfrqPin &mfrq_pin) : mfrq_pin_(mfrq_pin) {
  RtcSync::getInstance(); // Make sure RTC singleton exists.
}

void TimerSynced::setupMfrq(const uint16_t rate_hz, const bool invert) {
  // Set parameters.
  rate_hz_ = rate_hz;
  invert_trigger_ = invert;
  prescaler_ = RtcSync::getInstance().findMinPrescalerFrq(rate_hz, top_);
  RtcSync::getInstance().computeFrq(rate_hz, kPrescalers[prescaler_], &top_);

  // Setup timer specific match frequency configuration.
  setupMfrqWaveform();

  // Setup output pin.
  setupWaveOutPin();
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
  DEBUG_PRINTLN("[TimerSynced]: Configure SENSE.");
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
    NVIC_SetPriority(EIC_IRQn, 0x02);
    NVIC_EnableIRQ(EIC_IRQn);
  }
}

bool TimerSynced::getPinValue(const uint8_t group, const uint8_t pin) const {
  return PORT->Group[group].IN.reg & (1 << pin);
}

bool TimerSynced::getWaveOutPinValue() const {
  return getPinValue(mfrq_pin_.group, mfrq_pin_.pin);
}

bool TimerSynced::hasDataReady() {
  if (data_ready_) {
    data_ready_ = false; // Reset data ready flag.
    return true;
  } else {
    return false;
  }
}

ros::Time TimerSynced::computeTimeLastTrigger() {
  is_triggered_ = false;
  const uint16_t trigger_in_second = trigger_num_ % rate_hz_;
  const uint32_t ticks = (trigger_in_second * 2 + 1) * (top_ + 1);
  return RtcSync::getInstance().computeTime(trigger_secs_, ticks,
                                            kPrescalers[prescaler_]);
}

void TimerSynced::syncRtc() { ovf_ticks_since_sync_ = 0; }

void TimerSynced::trigger() {
  // TODO(rikba): If the trigger happens just before the RTC seconds update the
  // seconds could already be updated here and the trigger time is screwed. One
  // solution could be DMAC or capturing the total seconds.
  trigger_secs_ = RtcSync::getInstance().getSecs();
  trigger_num_++;
  is_triggered_ = true;
}

void TimerSynced::overflow() {
  // +1 to account for setting counter from TOP to ZERO cycle.
  ovf_ticks_since_sync_ += top_ + 1;
}

void TimerSynced::handleEic() {
  if (EIC->INTFLAG.vec.EXTINT & (1 << (dr_pin_ % 16))) {
    data_ready_ = true;
  }

  EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(1 << (dr_pin_ % 16));
}
