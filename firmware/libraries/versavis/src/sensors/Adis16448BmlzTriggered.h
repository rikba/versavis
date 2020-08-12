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

#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

class Adis16448BmlzTriggered : public ImuSynced {

public:
  Adis16448BmlzTriggered(ros::NodeHandle *nh, TimerSynced *timer,
                         const uint16_t rate_hz, const uint8_t dr_port_group,
                         const uint8_t dr_pin, const uint8_t chip_select);
  void setupRos(char *frame_id, char *rate_topic, char *imu_topic,
                char *baro_topic, char *mag_topic, char *temp_topic);
  bool publish() override;

private:
  enum class CalibrationStatus { kInit, kRunning, kCalibrating, kResetAvg, kFinished, kCalibrated };

  Adis16448 imu_;

  // ROS
  ros::Publisher *mag_pub_ = NULL;
  ros::Publisher *baro_pub_ = NULL;
  ros::Publisher *temp_pub_ = NULL;

  sensor_msgs::FluidPressure *baro_msg_ = NULL;
  sensor_msgs::MagneticField *mag_msg_ = NULL;
  sensor_msgs::Temperature *temp_msg_ = NULL;

  ros::Time stamp_;

  CalibrationStatus calibration_ = CalibrationStatus::kInit;
  uint16_t smpl_prd_settings_;
};

#endif
