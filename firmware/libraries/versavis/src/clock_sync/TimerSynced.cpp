#include "clock_sync/TimerSynced.h"

#include "clock_sync/RtcSync.h"
#include "helper.h"

TimerSynced::TimerSynced() {
  RtcSync::getInstance(); // Make sure RTC singleton exists.
}

ros::Time TimerSynced::computeTimeLastTrigger() {
  is_triggered_ = false;
  return RtcSync::getInstance().computeTime(ovf_ticks_since_sync_ - top_ - 1,
                                            kPrescalers[prescaler_]);
}

void TimerSynced::syncRtc() {
  if (RtcSync::getInstance().getSecs() != rtc_secs_) {
    rtc_secs_ = RtcSync::getInstance().getSecs();
    ovf_ticks_since_sync_ = 0;
  }
}

void TimerSynced::pwmPulse() {
  is_triggered_ = true;
  overflow();
}

void TimerSynced::overflow() {
  // +1 to account for setting counter from TOP to ZERO cycle.
  ovf_ticks_since_sync_ += top_ + 1;
}

uint8_t TimerSynced::findMinPrescaler(const uint16_t rate_hz,
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
