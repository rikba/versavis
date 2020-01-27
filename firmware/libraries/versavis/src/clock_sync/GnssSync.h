////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  GnssSync.h
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation to synchronize the VersaVIS against a GNSS receiver time.
//  This implementation assumes that the GNSS receiver is connected via PPS
//  signal and NMEA absolute time.
//  The PPS input signal is assumed to be received on SAMD pin PA11.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GnssSync_h
#define GnssSync_h

#include <cstdint>

#include <Uart.h>

#include <ros.h>
#include <versavis/ExtClkFilterState.h>

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

  void setMeasurementNoise(const float R_tps);
  void setProcessNoise(const float Q_tps);

  inline bool valid() { return !reset_time_; }

  void update();
  void reset();
  void getTimeNow(uint32_t *sec, uint32_t *nsec);
  ros::Time getTimeNow();

  inline void incrementPPS() { pps_cnt_++; }
  inline void measureTicksPerSecond(const uint32_t z) { z_ = z; }

  inline versavis::ExtClkFilterState getFilterState() { return filter_state_; }

  static void computeTime(const versavis::ExtClkFilterState &filter_state,
                          const double ticks_to_nanoseconds,
                          const uint32_t ticks, uint32_t *sec, uint32_t *nsec);

  static void computeTime(const versavis::ExtClkFilterState &filter_state,
                          const double ticks_to_nanoseconds,
                          const uint32_t ticks, ros::Time *time);

private:
  GnssSync();

  void setupSerial(Uart *uart, const uint32_t baud_rate);
  void setupCounter();
  void setupInterruptPa14();
  void setupRos(ros::NodeHandle *nh);
  bool waitForNmea();
  void updateTps();
  void resetFilterState();

  // ROS
  ros::NodeHandle *nh_;
  ros::Publisher filter_state_pub_;
  versavis::ExtClkFilterState filter_state_;

  // States
  bool reset_time_ = true;
  bool clear_uart_ = true;
  Uart *uart_ = NULL;
  double ticks_to_nanoseconds_ = 0;

  // Volatile measurements
  volatile uint32_t pps_cnt_ = 0;
  volatile uint32_t z_ = 0;
};

#endif
