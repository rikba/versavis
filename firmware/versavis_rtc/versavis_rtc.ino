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
ros::NodeHandle nh;

// Sensors.
Adis16448BmlzTriggered *imu_ptr_;

void setup() {
#ifdef DEBUG
  while (!SerialUSB) { // wait for serial port to connect.
  }
#endif
  DEBUG_PRINTLN("Setup.");

  // Sensors
  static Adis16448BmlzTriggered imu(&Tc3Synced::getInstance(), 10, PORTA, 13,
                                    10);
  imu_ptr_ = &imu;

/* ----- ROS ----- */
#ifndef DEBUG
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  while (!SerialUSB)
    ;

  RtcSync::getInstance().setupRos(&nh, "/versavis/rtc");
  imu.setupRos(&nh, "/versavis/imu");
#endif
}

void loop() {
  if (imu_ptr_)
    imu_ptr_->publish();

  //RtcSync::getInstance().publish();

#ifndef DEBUG
  nh.spinOnce();
#endif
}

void EIC_Handler() { Tc3Synced::getInstance().handleEic(); }
