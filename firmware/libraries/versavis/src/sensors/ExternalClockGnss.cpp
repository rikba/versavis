#include "sensors/ExternalClockGnss.h"

#include "helper.h"

ExternalClockGnss::ExternalClockGnss(Uart *uart, const uint32_t baud_rate)
    : ExternalClock(), uart_(uart) {
  DEBUG_PRINT("[ExternalClockGnss]: Setup serial connection with baud rate ");
  DEBUG_PRINTLN(baud_rate);
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

bool ExternalClockGnss::setRemoteTime() {
  // Try to get time 200ms after last PPS and 200ms before next PPS.
  // auto now = RtcSync::getInstance().getTimeNow();
  clock_msg_->remote_time = RtcSync::getInstance().getTimeNow();
  return true;
}
