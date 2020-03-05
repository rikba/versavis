#include "sensors/ExternalClockGnss.h"

#include <RTClib.h>

#include "clock_sync/RtcSync.h"

ExternalClockGnss::ExternalClockGnss(Uart *uart, const uint32_t baud_rate)
    : ExternalClock(), uart_(uart) {
  DEBUG_PRINT("[ExternalClockGnss]: Setup serial connection with baud rate ");
  DEBUG_PRINTLN(baud_rate);
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

ExternalClock::RemoteTimeStatus ExternalClockGnss::setRemoteTime() {
  RemoteTimeStatus result = RemoteTimeStatus::kWaiting;

  // Try to get time 200ms after last PPS and 200ms before next PPS.
  if (clock_msg_) {
    auto now = RtcSync::getInstance().getTimeNow();
    auto duration_since_pulse = computeDuration(clock_msg_->receive_time, now);

    auto duration_sec = duration_since_pulse.toSec();

    if (duration_sec < 0.0 || duration_sec > 0.8) {
      result = RemoteTimeStatus::kTimeout;
    } else if (duration_sec > 0.2) {
      result = RemoteTimeStatus::kReading;
    }
  }

  if (result == RemoteTimeStatus::kReading) {
    while (uart_ && uart_->available()) {
      auto nmea_result = nmea_parser_.parseChar(uart_->read());
      bool received_time = (nmea_result == NmeaParser::SentenceType::kGpZda) &&
                           (nmea_parser_.getGpZdaMessage().hundreths == 0);
      if (received_time && result == RemoteTimeStatus::kReceived) {
        // Ensure that the time on the serial is definitly the current time.
        // Received a second time from the buffer. Clear buffer and reject time.
        // TODO(rikba): This rejection scheme only works if there definitly is a
        // second time on the buffer. Could fail if the GNSS receiver outputs
        // more than GPZDA message.
        while (uart_->available())
          uart_->read(); // Clear UART buffer.
        result = RemoteTimeStatus::kTimeout;
        return result;
      } else if (received_time) {
        result = RemoteTimeStatus::kReceived;
      }
    }
  }

  if (result == RemoteTimeStatus::kReceived) {
    DateTime date_time(nmea_parser_.getGpZdaMessage().year,
                       nmea_parser_.getGpZdaMessage().month,
                       nmea_parser_.getGpZdaMessage().day,
                       nmea_parser_.getGpZdaMessage().hour,
                       nmea_parser_.getGpZdaMessage().minute,
                       nmea_parser_.getGpZdaMessage().second);
    if (clock_msg_) {
      clock_msg_->remote_time = ros::Time(date_time.unixtime(), 0);
    }
  }

  return result;
}
