#include "sensors/ExternalClockGnss.h"

#include <RTClib.h>

#include "clock_sync/RtcSync.h"

ExternalClockGnss::ExternalClockGnss(ros::NodeHandle *nh, Uart *uart,
                                     const uint32_t baud_rate)
    : ExternalClock(nh), uart_(uart) {
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

ExternalClock::RemoteTimeStatus ExternalClockGnss::setRemoteTime() {

  switch (result_) {
  case RemoteTimeStatus::kWaiting: {
    // Try to get time 200ms after last PPS and 200ms before next PPS.
    if (clock_msg_) {
      auto now = RtcSync::getInstance().getTimeNow();
      auto duration_since_pulse = now - clock_msg_->receive_time;

      auto duration_sec = duration_since_pulse.toSec();

      if (duration_sec < -RtcSync::getInstance().getTimeResolution().toSec() ||
          duration_sec > 0.8) {
        if (nh_) {
          char warning[255];
          sprintf(warning,
                  "Timeout waiting for GNSS time. Now: %.9f, Received: %.9f, "
                  "Duration: %.9f",
                  now.toSec(), clock_msg_->receive_time.toSec(), duration_sec);
          nh_->logwarn(warning);
        }
        result_ = RemoteTimeStatus::kTimeout;
      } else if (duration_sec > 0.2) {
        result_ = RemoteTimeStatus::kReading;
      }
    }
    break;
  }
  case RemoteTimeStatus::kReading: {
    if (uart_ && uart_->available()) {
      auto nmea_result = nmea_parser_.parseChar(uart_->read());
      bool received_time = (nmea_result == NmeaParser::SentenceType::kGpZda) &&
                           (nmea_parser_.getGpZdaMessage().hundreths == 0);
      // TODO(rikba): Actually the nmea parser should return an empty buffer.
      if (received_time && uart_->available() > 1) {
        // Ensure that the time on the serial is definitly the current time.
        // There is more on the buffer left after receiving the time.
        if (nh_) {
          char warning[255];
          sprintf(warning,
                  "Data remaining on serial buffer after receiving GNSS time. "
                  "Buffer size: %d",
                  uart_->available());
          nh_->logwarn(warning);
        }
        while (uart_->available()) {
          uart_->read(); // Clear UART buffer.
        }
        result_ = RemoteTimeStatus::kTimeout;
      } else if (received_time) {
        DateTime date_time(nmea_parser_.getGpZdaMessage().year,
                           nmea_parser_.getGpZdaMessage().month,
                           nmea_parser_.getGpZdaMessage().day,
                           nmea_parser_.getGpZdaMessage().hour,
                           nmea_parser_.getGpZdaMessage().minute,
                           nmea_parser_.getGpZdaMessage().second);
        if (clock_msg_) {
          clock_msg_->remote_time = ros::Time(date_time.unixtime(), 0);
        }
        // TODO(rikba): Actually the nmea parser should return an empty buffer.
        while (uart_->available()) {
          nmea_parser_.parseChar(uart_->read()); // Clear UART buffer.
        }
        result_ = RemoteTimeStatus::kReceived;
      }
    } else {
      auto now = RtcSync::getInstance().getTimeNow();
      auto duration_since_pulse = now - clock_msg_->receive_time;
      auto duration_sec = duration_since_pulse.toSec();
      if (duration_sec > 0.8) {
        if (nh_) {
          nh_->logerror("Timeout. No data on GNSS time buffer.");
        }
        result_ = RemoteTimeStatus::kTimeout;
      }
    }
    break;
  }
  case RemoteTimeStatus::kReceived: {
    result_ = RemoteTimeStatus::kWaiting;
    break;
  }
  case RemoteTimeStatus::kTimeout: {
    result_ = RemoteTimeStatus::kWaiting;
    break;
  }
  default: {
    result_ = RemoteTimeStatus::kWaiting;
    break;
  }
  }

  return result_;
}
