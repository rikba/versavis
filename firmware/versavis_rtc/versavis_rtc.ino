#include "helper.h"
#include "versavis_configuration.h"

#include "clock_sync/RtcSync.h"
#include "clock_sync/Tc3Synced.h"
#include "clock_sync/Tc4Synced.h"
#include "clock_sync/Tc5Synced.h"
#include "clock_sync/Tcc0Synced.h"
#include "clock_sync/Tcc2Synced.h"

#include "sensors/Adis16448BmlzTriggered.h"
#include "sensors/CamSyncedExposure.h"
#include "sensors/ExternalClockGnss.h"
#include "sensors/LidarLite.h"

#include <Arduino.h>

// ROS
ros::NodeHandle *nh = NULL;

// Sensors.
Adis16448BmlzTriggered *imu = NULL;
CamSyncedExposure *cam0 = NULL;
ExternalClock *ext_clock = NULL;
LidarLite *lidar = NULL;

void setup() {
#ifndef DEBUG
  static ros::NodeHandle node_handle;
  nh = &node_handle;
  nh->initNode();
  nh->loginfo("Node initialized.");
#endif
  while (!SerialUSB)
    ;

  DEBUG_PRINTLN("Setup.");

  // Sensors
  static Adis16448BmlzTriggered adis_16448(nh, &Tc3Synced::getInstance(), 100,
                                           PORTA, 13, 10);
  imu = &adis_16448;

  static CamSyncedExposure bfly(nh, &Tcc0Synced::getInstance(), 15, false,
                                true);
  cam0 = &bfly;

  static ExternalClockGnss gnss(nh, &Serial, 115200);
  ext_clock = &gnss;

  static LidarLite lidar_lite(nh, &Tc4Synced::getInstance(), 10);
  lidar = &lidar_lite;

  // ROS
  static char *rtc_topic = "/versavis/rtc";
  RtcSync::getInstance().setupRos(nh, rtc_topic);

  static char *baro_topic = "/versavis/adis16448/baro";
  static char *imu_topic = "/versavis/adis16448/imu";
  static char *mag_topic = "/versavis/adis16448/mag";
  static char *temp_topic = "/versavis/adis16448/temp";
  imu->setupRos(baro_topic, imu_topic, mag_topic, temp_topic);

  static char *bfly_topic = "/versavis/bfly/image";
  cam0->setupRos(bfly_topic);

  static char *ext_clock_topic = "/versavis/gnss/time_sync";
  ext_clock->setupRos(ext_clock_topic);

  static char *lidar_lite_topic = "/versavis/lidar_lite/range";
  lidar->setupRos(lidar_lite_topic);
}

void loop() {
  imu->publish();
  cam0->publish();
  ext_clock->publish();
  lidar_lite->publish();

  RtcSync::getInstance().publish();

  if (nh)
    nh->spinOnce();
}

void EIC_Handler() {
  RtcSync::getInstance().handleEic();
  Tc3Synced::getInstance().handleEic();
}
