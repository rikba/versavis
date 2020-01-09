#include "GnssSync.h"

#include <MicroNMEA.h>
#include <RTClib.h>

#include "helper.h"
#include "versavis_configuration.h"

GnssSync::GnssSync(Uart *uart, const uint8_t timeout_nmea_s /*= 30*/,
                   const double R_tps /*= 100.0*/, const double Q_tps /*= 1.0*/)
    : uart_(uart), timeout_nmea_s_(timeout_nmea_s), R_tps_(R_tps),
      Q_tps_(Q_tps) {}

void GnssSync::setup(const uint32_t baud_rate /*= 115200*/,
                     const uint8_t pps_pin /*= 3*/) {
  setupSerial(baud_rate);
  setupCounter(pps_pin);
}

void GnssSync::update() {
  if (reset_time_) {
    waitForNmea();
  }
}

void GnssSync::reset() { reset_time_ = true; }

void GnssSync::getTimeNow(uint32_t *sec, uint32_t *nsec) {
  if (sec) {
    *sec = t_nmea_ + pps_cnt_;
  }
  if (nsec) {
    //  *nsec = double(REG_TC4_COUNT32_COUNT) * x_nspt_;
  }
}

void GnssSync::setupSerial(const uint32_t baud_rate) {
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

void GnssSync::setupCounter(const uint8_t pps_pin) {}

void waitForNmea() {
  reset_time_ = false;
  char nmea_buffer[255];
  MicroNMEA nmea(nmea_buffer, sizeof(nmea_buffer));

  bool time_valid = false;
  uint32_t start_time = millis();
  uint32_t duration_s = 0;
  uint32_t t_nmea_pps_cnt = pps_cnt_; // Assign pps signal to time.

  // Find the unix time that belongs to the last pps pulse.
  while (!time_valid && duration_s < timeout_nmea_s_) {
    t_nmea_pps_cnt = pps_cnt_;
    while (Serial.available()) {
      nmea.process(Serial.read());
    }
    time_valid = nmea.getYear() != 0 && nmea.getHour() != 99 &&
                 nmea.getHundredths() == 0;
  }

  // Save the time when pps counting started.
  DateTime date_time;
  if (time_valid) {
    date_time = DateTime(nmea.getYear(), nmea.getMonth(), nmea.getDay(),
                         nmea.getHour(), nmea.getMinute(), nmea.getSecond());
  }
  t_nmea_ = date_time.unixtime() - t_nmea_pps_cnt;

#ifdef DEBUG
  if (time_valid) {
    DEBUG_PRINTLN("Received NMEA time.");
  }
  DEBUG_PRINTLN("Time at pps_cnt == 0: " + t_nmea_);
#endif
}
