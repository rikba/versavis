////////////////////////////////////////////////////////////////////////////////
//  April 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  LidarLite.h
////////////////////////////////////////////////////////////////////////////////
//
// Implements the I2C Lidar Lite interface. The Lidar is triggered and time
// stamped using the Mode Pin and then read using the I2C connection.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_LidarLite_h
#define Sensors_LidarLite_h

#include <versavis/LidarLite.h>
#include "sensors/SensorSynced.h"

class LidarLite : public SensorSynced {
public:
  LidarLite(ros::NodeHandle *nh, TimerSynced *timer, const uint16_t rate_hz);
  void setupRos(const char *topic) override;
  bool publish() override;

private:
  uint32_t last_msg_ = 0xFFFFFFFF;

  // I2C interface.
  bool readData(versavis::LidarLite *msg) const;
  bool readSignalStrength(uint8_t *signal_strength) const;
  uint8_t write(uint8_t reg_adr, uint8_t data) const;
  bool busy() const;

  versavis::LidarLite *msg_ = NULL;
};

#endif
