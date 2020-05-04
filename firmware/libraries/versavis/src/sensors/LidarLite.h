////////////////////////////////////////////////////////////////////////////////
//  April 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  LidarLite.h
////////////////////////////////////////////////////////////////////////////////
//
// Implements the I2C Lidar Lite interface.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_LidarLite_h
#define Sensors_LidarLite_h

#include "sensors/RangeSynced.h"

class LidarLite : public RangeSynced {
public:
  LidarLite(ros::NodeHandle *nh, TimerSynced *timer, const uint16_t rate_hz);
  void publish() override;

private:
  uint16_t last_msg_ = 0xFFFF;

  // I2C interface.
  uint8_t readDistance(uint16_t *distance) const;
  uint8_t write(uint8_t reg_adr, uint8_t data) const;
  bool busy() const;
};

#endif
