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
#include "sensors/UsD1.h"

#include <Arduino.h>

#define IMU_FRAME_ID "adis16448bmlz"
#define IMU_RATE_TOPIC "adis16448bmlz/imu/set_rate"
#define IMU_IMU_TOPIC "adis16448bmlz/imu/data_raw"
#define IMU_BARO_TOPIC "adis16448bmlz/imu/baro"
#define IMU_MAG_TOPIC "adis16448bmlz/imu/mag"
#define IMU_TEMP_TOPIC "adis16448bmlz/imu/temp"

#define LIDAR_FRAME_ID "lidar_lite"
#define LIDAR_RATE_TOPIC "lidar_lite/set_rate"
#define LIDAR_DATA_TOPIC "lidar_lite/data"

#define CAM_FRAME_ID "bfly"
#define CAM_RATE_TOPIC "bfly/set_rate"
#define CAM_SEQ_TOPIC "bfly/set_seq"
#define CAM_IMG_TOPIC "bfly/header"

#define RADAR_FRAME_ID "us_d1"
#define RADAR_DATA_TOPIC "us_d1/data"

// ROS
ros::NodeHandle *nh = NULL;

// Sensors.
Adis16448BmlzTriggered *imu = NULL;
CamSyncedExposure *cam0 = NULL;
ExternalClock *ext_clock = NULL;
LidarLite *lidar = NULL;
UsD1 *radar = NULL;

void setup() {
#ifndef DEBUG
  static ros::NodeHandle node_handle;
  nh = &node_handle;
  nh->initNode();
  nh->loginfo("Node initialized.");
#endif
  while (!SerialUSB)
    ;

  // Sensors
  static Adis16448BmlzTriggered adis_16448(nh, &Tc3Synced::getInstance(), 500,
                                           PORTA, 13, 10);
  imu = &adis_16448;

  static CamSyncedExposure bfly(nh, &Tcc0Synced::getInstance(), 10, false,
                                true);
  cam0 = &bfly;

  static ExternalClockGnss gnss(nh, &Serial, 115200);
  ext_clock = &gnss;

  static UsD1 us_d1(nh, &Serial1);
  radar = &us_d1;

  static LidarLite lidar_lite(nh, &Tc4Synced::getInstance(), 100);
  lidar = &lidar_lite;

  // ROS
  RtcSync::getInstance().setupRos(nh);

  imu->setupRos(IMU_FRAME_ID, IMU_RATE_TOPIC, IMU_IMU_TOPIC, IMU_BARO_TOPIC,
                IMU_MAG_TOPIC, IMU_TEMP_TOPIC);

  cam0->setupRos(CAM_FRAME_ID, CAM_RATE_TOPIC, CAM_SEQ_TOPIC, CAM_IMG_TOPIC);

  ext_clock->setupRos();
  radar->setupRos(RADAR_FRAME_ID, RADAR_DATA_TOPIC);
  lidar->setupRos(LIDAR_FRAME_ID, LIDAR_RATE_TOPIC, LIDAR_DATA_TOPIC);
}

void loop() {
  // Prioritizing readout queue.
  if (imu->publish())
    ;
  else if (cam0->publish())
    ;
  else if (radar->publish())
    ;
  else if (lidar->publish())
    ;
  else if (ext_clock->publish())
    ;
  else if (RtcSync::getInstance().publish())
    ;
  else if (nh) { // Only spin if no sensor is busy.
    nh->spinOnce();
  }
}

void EIC_Handler() {
  RtcSync::getInstance().handleEic();
  Tc3Synced::getInstance().handleEic();
}
