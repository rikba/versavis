#include "clock_sync/TimerSynced.h"

#include "clock_sync/RtcSync.h"
#include "helper.h"

TimerSynced::TimerSynced() {
  RtcSync::getInstance(); // Make sure RTC singleton exists.
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
  // seconds could already be updated here and the triggering is screwed. One
  // solution could be DMAC or capturing the total seconds.
  trigger_secs_ = RtcSync::getInstance().getSecs();
  trigger_num_++;
  is_triggered_ = true;
}

void TimerSynced::overflow() {
  // +1 to account for setting counter from TOP to ZERO cycle.
  ovf_ticks_since_sync_ += top_ + 1;
}

uint8_t TimerSynced::findMinPrescalerPwm(const uint16_t rate_hz,
                                         const uint32_t clock_freq,
                                         const uint32_t counter_max) {
  bool found_prescaler = false;
  uint8_t prescaler = 0;
  for (; prescaler < sizeof(kPrescalers) / sizeof(kPrescalers[0]);
       ++prescaler) {
    const uint32_t kRequiredTicks =
        clock_freq / kPrescalers[prescaler] / static_cast<uint32_t>(rate_hz);
    found_prescaler = (kRequiredTicks < counter_max);
    if (found_prescaler) {
      break;
    }
  }

  if (!found_prescaler) {
    error("NO_PRESCALER (TimerSynced.cpp): cannot find suitable prescaler.",
          201);
  }

  return prescaler;
}

uint8_t TimerSynced::findMinPrescalerFrq(const uint16_t rate_hz,
                                         const uint32_t clock_freq,
                                         const uint32_t counter_max) {
  return findMinPrescalerPwm(rate_hz, clock_freq / 2, counter_max);
}
