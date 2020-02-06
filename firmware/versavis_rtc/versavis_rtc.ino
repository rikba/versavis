#include "helper.h"
#include "versavis_configuration.h"

#include "clock_sync/RtcSync.h"
#include "clock_sync/Tc3Synced.h"
#include "clock_sync/Tc4Synced.h"
#include "clock_sync/Tc5Synced.h"
#include "clock_sync/Tcc0Synced.h"
#include "clock_sync/Tcc2Synced.h"

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
  Tc3Synced::getInstance().setupMfrq(10, false);
  Tc3Synced::getInstance().setupDataReady(PORTA, 13, InterruptLogic::kRise);
}

void loop() {
  // Synchronize timers against RTC clock.
  if (Tc3Synced::getInstance().isTriggered()) {
    auto t3 = Tc3Synced::getInstance().computeTimeLastTrigger();

    DEBUG_PRINT("triggered t3: ");
    DEBUG_PRINT(t3.sec);
    DEBUG_PRINT(".");
    DEBUG_PRINTDECLN(t3.nsec);
  }
  //DEBUG_PRINTLN(PORT->Group[PORTA].IN.reg & (1 << 13));

if (Tc3Synced::getInstance().hasDataReady()) {
  DEBUG_PRINTLN("t3 data ready.");
}

  //  if (Tc4Synced::getInstance().isTriggered()) {
  //    auto t4 = Tc4Synced::getInstance().computeTimeLastTrigger();
  //
  //    DEBUG_PRINT("t4: ");
  //    DEBUG_PRINT(t4.sec);
  //    DEBUG_PRINT(".");
  //    DEBUG_PRINTDECLN(t4.nsec);
  //  }
  //  if (Tc5Synced::getInstance().isTriggered()) {
  //    auto t5 = Tc5Synced::getInstance().computeTimeLastTrigger();
  //
  //    DEBUG_PRINT("t5: ");
  //    DEBUG_PRINT(t5.sec);
  //    DEBUG_PRINT(".");
  //    DEBUG_PRINTDECLN(t5.nsec);
  //  }
  // if (Tcc0Synced::getInstance().isTriggered()) {
  //   auto tcc0 = Tcc0Synced::getInstance().computeTimeLastTrigger();
  //
  //   DEBUG_PRINT("tcc0: ");
  //   DEBUG_PRINT(tcc0.sec);
  //   DEBUG_PRINT(".");
  //   DEBUG_PRINTDECLN(tcc0.nsec);
  // }
  // if (Tcc2Synced::getInstance().isTriggered()) {
  //   auto tcc2 = Tcc2Synced::getInstance().computeTimeLastTrigger();
  //
  //   DEBUG_PRINT("tcc2: ");
  //   DEBUG_PRINT(tcc2.sec);
  //   DEBUG_PRINT(".");
  //   DEBUG_PRINTDECLN(tcc2.nsec);
  // }

#ifndef DEBUG
  nh.spinOnce();
#endif
}

void EIC_Handler() {
  Tc3Synced::getInstance().handleEic();
}
