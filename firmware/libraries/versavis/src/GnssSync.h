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

#include <ros.h>
#include <versavis/ExtClkFilterState.h>

#include "versavis_configuration.h"

class Timestamp {
public:
  inline bool hasTime() { return has_time_; }
  bool getTime(uint32_t *sec, uint32_t *nsec);
  void setTime(const versavis::ExtClkFilterState &filter_state,
               const uint32_t ticks);

private:
  versavis::ExtClkFilterState filter_state_;
  uint32_t ticks_ = 0;
  bool has_time_ = false;
};

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
  void setup(ros::NodeHandle *nh, Uart *uart,
             const uint32_t baud_rate = 115200);

  void setTimeoutNmea(const uint8_t timeout_nmea_s);
  void setMeasurementNoise(const double R_tps);
  void setProcessNoise(const double Q_tps);

  void update();
  void reset();
  void getTimeNow(uint32_t *sec, uint32_t *nsec);

  inline void incrementPPS() { filter_state_.pps_cnt++; }
  inline void measureTicksPerSecond(const uint32_t z) { filter_state_.z = z; }

  inline versavis::ExtClkFilterState getFilterState() { return filter_state_; }

  bool getTimePa14(uint32_t *sec, uint32_t *nsec);
  void setTimePa14(const uint32_t ticks);

  static void computeTime(const versavis::ExtClkFilterState &filter_state,
                          const uint32_t ticks, uint32_t *sec, uint32_t *nsec);

  static void computeTime(const versavis::ExtClkFilterState &filter_state,
                          const uint32_t ticks, ros::Time *time);

private:
  GnssSync();

  void setupSerial(Uart *uart, const uint32_t baud_rate);
  void setupCounter();
  void setupInterruptPa14();
  void setupRos(ros::NodeHandle *nh);
  void waitForNmea();
  void updateTps();
  void resetFilterState();

  // Parameters
  // Timeout to wait for NMEA absolute time.
  uint8_t timeout_nmea_s_ = 30.0;

  // ROS
  ros::NodeHandle *nh_;
  ros::Publisher filter_state_pub_;
  versavis::ExtClkFilterState filter_state_;

  // States
  bool reset_time_ = true;
  Uart *uart_;

  // External events.
  Timestamp timestamp_pa14_;
};

#endif
