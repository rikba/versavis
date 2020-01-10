////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  PpsSync.h
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation to synchronize the VersaVIS against a GNSS receiver time.
//  This implementation assumes that the GNSS receiver is connected via PPS
//  signal and NMEA absolute time.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GnssSync_h
#define GnssSync_h

#include <Uart.h>

#include "versavis_configuration.h"

class GnssSync {
public:
  GnssSync(Uart *uart, const uint8_t timeout_nmea_s_ = 30,
           const double R_tps = 100.0, const double Q_tps = 1.0);
  void setup(const uint32_t baud_rate = 115200, const uint8_t pps_pin = 3);
  void update();
  void reset();
  void getTimeNow(uint32_t *sec, uint32_t *nsec);

private:
  void setupSerial(const uint32_t baud_rate);
  void setupCounter(const uint8_t pps_pin);
  void waitForNmea();

  // Parameters
  // Timeout to wait for NMEA absolute time.
  const uint8_t timeout_nmea_s_;
  // Measurement noise covariance. [Ticks^2/second^2]
  const double R_tps_;
  // Process noise covariance. [Ticks^2/second^2]
  const double Q_tps_;

  // States
  bool reset_time_ = true;
  Uart *uart_;

  // PPS update.
  volatile uint32_t pps_cnt_ = 0;
  volatile uint32_t tps_meas_ = 0;
  uint32_t pps_cnt_prev = pps_cnt_;
  uint32_t t_nmea_ = 0;

  // Ticks per second Kalman filter.
#ifdef USE_GCLKIN_10MHZ
  double x_tps_ = 10000000.0;
#else
  double x_tps_ = 32768.0;
#endif
  double P_tps_ = R_tps_; // Tick prior covariance. [Ticks^2/second^2]
  double x_nspt_ = 1000000000.0 / x_tps_; // Nanoseconds per tick.
};

#endif
