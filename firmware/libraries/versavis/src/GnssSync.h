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
  // Singleton implementation.
  inline static GnssSync &getInstance() {
    static GnssSync instance;
    return instance;
  }
  GnssSync(GnssSync const &) = delete;
  void operator=(GnssSync const &) = delete;

  // pps_pin_samd_io can be 11, 14 or 27 on AUX connector.
  void setup(Uart *uart, const uint32_t baud_rate = 115200);

  void setTimeoutNmea(const uint8_t timeout_nmea_s);
  void setMeasurementNoise(const double R_tps);
  void setProcessNoise(const double Q_tps);

  void update();
  void reset();
  void getTimeNow(uint32_t *sec, uint32_t *nsec);

  inline void incrementPPS() { pps_cnt_++; }
  inline void measureTicksPerSecond(const uint32_t tps_meas) {
    tps_meas_ = tps_meas;
  }

  uint32_t getPpsCnt();
  uint32_t getTpsMeas();

private:
  GnssSync() {}

  void setupSerial(Uart *uart, const uint32_t baud_rate);
  void setupCounter();
  void setupInterruptPa14();
  void waitForNmea();
  void updateTps();

  // Parameters
  // Timeout to wait for NMEA absolute time.
  uint8_t timeout_nmea_s_ = 30.0;
  // Kalman filter variables to estimate ticks per second x_tps_.
  // R_tps_: Measurement noise covariance. [Ticks^2/second^2]
  // Q_tps_: Process noise covariance. [Ticks^2/second^2]

#ifdef USE_GCLKIN_10MHZ
  double x_tps_ = 10000000.0;
  double R_tps_ = 100.0;
  double Q_tps_ = 1.0;
#elif defined USE_DFLL48M
  double x_tps_ = 48000000.0;
  double R_tps_ = 10000.0;
  double Q_tps_ = 100.0;
#else
  double x_tps_ = 32768.0;
  double R_tps_ = 100.0;
  double Q_tps_ = 1.0;
#endif

  // States
  bool reset_time_ = true;
  Uart *uart_;

  // PPS update.
  volatile uint32_t pps_cnt_ = 0;
  volatile uint32_t tps_meas_ = 0;
  uint32_t pps_cnt_prev_ = pps_cnt_;
  uint32_t t_nmea_ = 0;

  double P_tps_ = R_tps_; // Tick prior covariance. [Ticks^2/second^2]
  double x_nspt_ = 1000000000.0 / x_tps_; // Nanoseconds per tick.
};

#endif
