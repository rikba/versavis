#include "helper.h"
#include "versavis_configuration.h"

#include "clock_sync/RtcSync.h"
#include "sensors/Tc3Synced.h"
#include "sensors/Tc4Synced.h"
#include "sensors/Tc5Synced.h"
#include "sensors/Tcc0Synced.h"
#include "sensors/Tcc2Synced.h"

#include <Arduino.h>

/* ----- ROS ----- */
ros::NodeHandle nh;

void setup() {
  DEBUG_PRINTLN("Setup.");

/* ----- ROS ----- */
#ifndef DEBUG
  nh.getHardware()->setBaud(250000);
  nh.initNode();

  RtcSync::getInstance().setupRos(nh);
  nh.spinOnce();
#else
  while (!SerialUSB) { // wait for serial port to connect.
  }
#endif

  /* ----- Timers ----- */
  Tc3Synced::getInstance().setup();
  Tc4Synced::getInstance().setup();
  Tc5Synced::getInstance().setup();
  Tcc0Synced::getInstance().setup();
  Tcc2Synced::getInstance().setup();
}

void loop() {
  DEBUG_PRINTLN("Loop.");
  // Synchronize timers against RTC clock.
  Tc3Synced::getInstance().sync();
  Tc4Synced::getInstance().sync();
  Tc5Synced::getInstance().sync();
  Tcc0Synced::getInstance().sync();
  Tcc2Synced::getInstance().sync();

  delay(100);
#ifndef DEBUG
  nh.spinOnce();
#endif
}
