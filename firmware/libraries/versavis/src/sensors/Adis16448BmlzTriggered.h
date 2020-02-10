////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Adis16448BmlzTriggered.h
////////////////////////////////////////////////////////////////////////////////
//
//  Triggered Adis16448 implementation.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_Adis16448BmlzTriggered_h
#define Sensors_Adis16448BmlzTriggered_h

#include "drivers/Adis16448.h"
#include "sensors/ImuSynced.h"

#include <versavis/MagneticMicro.h>
#include <versavis/PressureMicro.h>
#include <versavis/TemperatureMicro.h>

class Adis16448BmlzTriggered : public ImuSynced {
public:
  Adis16448BmlzTriggered(TimerSynced *timer, const uint16_t rate_hz,
                         const uint8_t dr_port_group, const uint8_t dr_pin,
                         const uint8_t chip_select);
  void setupRos(ros::NodeHandle *nh, const char *baro_topic,
                const char *imu_topic, const char *mag_topic,
                const char *temp_topic);
  void publish() override;

private:
  Adis16448 imu_;

  // ROS
  ros::Publisher *mag_pub_ = NULL;
  ros::Publisher *baro_pub_ = NULL;
  ros::Publisher *temp_pub_ = NULL;

  versavis::MagneticMicro *mag_msg_ = NULL;
  versavis::PressureMicro *baro_msg_ = NULL;
  versavis::TemperatureMicro *temp_msg_ = NULL;
};

#endif
