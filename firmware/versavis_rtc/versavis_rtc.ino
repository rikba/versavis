#include "helper.h"
#include "versavis_configuration.h"

#include "clock_sync/RtcSync.h"
#include "clock_sync/Tc3Synced.h"
#include "clock_sync/Tc4Synced.h"
#include "clock_sync/Tc5Synced.h"
#include "clock_sync/Tcc0Synced.h"
#include "clock_sync/Tcc2Synced.h"

#include "sensors/Adis16448BmlzTriggered.h"

#include <Arduino.h>

// ROS
ros::NodeHandle *nh = NULL;

// Sensors.
SensorSynced *imu = NULL;

void setup() {
#ifndef DEBUG
  static ros::NodeHandle node_handle;
  nh = &node_handle;
  nh->initNode();
#endif
  while (!SerialUSB)
    ;

  DEBUG_PRINTLN("Setup.");

  // Sensors
  static Adis16448BmlzTriggered adis_16448(&Tc3Synced::getInstance(), 1000,
                                           PORTA, 13, 10);
  imu = &adis_16448;

  // ROS
  RtcSync::getInstance().setupRos(nh, "/versavis/rtc");
  imu->setupRos(nh, "/versavis/imu");
}

void loop() {
  imu->publish();

  // RtcSync::getInstance().publish();

  if (nh)
    nh->spinOnce();
}

void EIC_Handler() { Tc3Synced::getInstance().handleEic(); }
