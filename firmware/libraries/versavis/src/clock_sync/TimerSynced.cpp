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

bool TimerSynced::getWaveOutPinValue() const {
  return PORT->Group[mfrq_pin_.group].IN.reg & (1 << mfrq_pin_.pin);
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
