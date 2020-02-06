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
