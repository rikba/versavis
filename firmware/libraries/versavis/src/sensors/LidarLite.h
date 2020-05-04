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

#include "image_numbered_msgs/LidarLite.h"
#include "sensors/SensorSynced.h"

class LidarLite : public SensorSynced {
public:
  LidarLite(ros::NodeHandle *nh, TimerSynced *timer, const uint16_t rate_hz);
  void setupRos(const char *topic) override;
  void publish() override;

private:
  uint16_t last_msg_ = 0xFFFF;

  // I2C interface.
  bool readData(image_numbered_msgs::LidarLite *msg) const;
  bool readSignalStrength(uint8_t *signal_strength) const;
  uint8_t write(uint8_t reg_adr, uint8_t data) const;
  bool busy() const;

  image_numbered_msgs::LidarLite *msg_ = NULL;
};

#endif
