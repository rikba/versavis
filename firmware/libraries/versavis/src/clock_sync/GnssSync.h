////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  GnssSync.h
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation to synchronize the VersaVIS against a GNSS receiver time
//  using TCC1.
//
//  - Timer start on receiving PPS pulse
//  - Hard reset RTC total time on first PPS pulse
//  - Capture PPS on CC0
//  - Capture RTC OvF on MC0
//  - Read NMEA on UART
//  - Kalman filter to estimate RTC offset and skew
//  - Control RTC offset and skew by modifying RTC COMP0 register.
//  - Optional: Control clock skew by sending out analog signal.
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

  void setup(ros::NodeHandle *nh, Uart *uart,
             const uint32_t baud_rate = 115200);
  // Setup the PPS period counter on TC4/5.
  void setupCounter();

  void setMeasurementNoise(const float R_tps);
  void setProcessNoise(const float Q_tps);

  inline bool valid() { return !reset_time_; }

  void update();
  void reset();
  ros::Time getTimeNow();

  inline void incrementPPS() { pps_cnt_++; }
  inline void measureTicksPerSecond(const uint32_t z) { z_ = z; }

  inline versavis::ExtClkFilterState getFilterState() { return filter_state_; }

  static void computeTime(const versavis::ExtClkFilterState &filter_state,
                          const double ticks_to_nanoseconds, int32_t ticks,
                          uint32_t *sec, uint32_t *nsec);

  static void computeTime(const versavis::ExtClkFilterState &filter_state,
                          const double ticks_to_nanoseconds, int32_t ticks,
                          ros::Time *time);

private:
  GnssSync();

  void setupSerial(Uart *uart, const uint32_t baud_rate);
  void setupInterruptPa14();
  void setupRos(ros::NodeHandle *nh);
  bool waitForNmea();
  void updateTps();
  void resetFilterState();

  void setupPort();
  void setupGenericClock4();
  void setupEic();
  void setupEvsys();
  void setupTCC0();

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
