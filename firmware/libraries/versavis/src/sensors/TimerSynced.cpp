#include "sensors/TimerSynced.h"

#include "clock_sync/RtcSync.h"

TimerSynced::TimerSynced() {
  RtcSync::getInstance(); // Make sure RTC singleton exists.
}

ros::Time TimerSynced::computeTimeLastTrigger() {
  is_triggered_ = false;
  return RtcSync::getInstance().computeTime(ovf_ticks_since_sync_,
                                            kPrescalers[prescaler_]);
}

void TimerSynced::retrigger() { ovf_ticks_since_sync_ = 0; }

void TimerSynced::overflow() {
  // +1 to account for setting counter from TOP to ZERO cycle.
  ovf_ticks_since_sync_ += top_ + 1;
}

void TimerSynced::handleInterrupt() {
  is_triggered_ = true;
  handleRetrigger();
  handleOverflow();
}
