#include "RtcSync.h"

#include <RTClib.h>

#include "helper.h"
#include "versavis_configuration.h"

RtcSync::RtcSync() : rtc_pub_("/versavis/gnss_sync/rtc", &rtc_msg_) {}

void RtcSync::setup(ros::NodeHandle *nh) {
  setupRos(nh);
  setupPort();
  setupGenericClock5();
  setupEvsys();
  setupRtc();
}

void RtcSync::setupRos(ros::NodeHandle *nh) {
  nh_ = nh;
#ifndef DEBUG
  if (nh_) {
    nh_->advertise(rtc_pub_);
  }
#endif
}

void RtcSync::setupPort() const {
  // Setup the external clock if necessary.
#ifdef RTC_GCLKIN_10MHZ
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.

  DEBUG_PRINTLN("[GnssSync]: Configuring PA10/GCLK_IO[4] as input.");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 10); // Set pin PA10 pin as input
  PORT->Group[PORTA].PMUX[10 >> 1].reg |=
      PORT_PMUX_PMUXE_H; // Connect PA10 pin to peripheral H (GCLK_IO[4])
  PORT->Group[PORTA].PINCFG[10].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[10].reg |= PORT_PINCFG_INEN; // Enable input
#endif
}

void RtcSync::setupGenericClock5() const {
  REG_PM_APBAMASK |= PM_APBAMASK_GCLK; // GCLK APB Clock Enable

  DEBUG_PRINTLN("[RtcSync]: Configuring GENCTRL register.");
#ifdef RTC_GCLKIN_10MHZ
  DEBUG_PRINTLN("[RtcSync]: Using external clock.");
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_GCLKIN |
                      GCLK_GENCTRL_ID(4) | GCLK_GENCTRL_RUNSTDBY;
#else
  DEBUG_PRINTLN("[RtcSync]: Using XOSC32K.");
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K |
                      GCLK_GENCTRL_ID(4) | GCLK_GENCTRL_RUNSTDBY;
#endif
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock 4 for RTC.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_RTC;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock 4 for EVSYS CH0.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_EVSYS_0;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
}

void RtcSync::setupEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS; // Enable EVSYS.

  DEBUG_PRINTLN("[RtcSync]: Connect all timers to this event on channel 0.");
  // TCC0
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0); // Start

  // TCC1
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_0); // Start
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_1); // Capt

  // TCC2
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC2_EV_0); // Start

  // TC3
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU); // Start

  // TC4
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU); // Start

  // TC5
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC5_EVU); // Start

  DEBUG_PRINTLN("[RtcSync]: Configuring EVSYS channel.");
  EVSYS->CHANNEL.reg =
      EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_RTC_CMP_0) | EVSYS_CHANNEL_CHANNEL(0);
  while (EVSYS->CHSTATUS.bit.CHBUSY0) {
  }
}

void RtcSync::setupRtc() const {
  REG_PM_APBAMASK |= PM_APBAMASK_RTC; // Enable RTC power manager.

  DEBUG_PRINTLN("[RtcSync]: Disabling RTC.");
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE; // Disable RTC.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: RTC software reset.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: RTC MODE0 COUNT32.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_MODE_COUNT32;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: RTC MATCHCLR.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_MATCHCLR;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Reset counter value.");
  RTC->MODE0.COUNT.reg = 0;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Set CMP0.");
  RTC->MODE0.COMP[0].reg = RTC_FREQ - 1;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Enable Interrupts.");
  RTC->MODE0.INTENSET.reg |= RTC_MODE0_INTENSET_CMP0;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Enabling CMPEO0 event.");
  RTC->MODE0.EVCTRL.reg |= RTC_MODE0_EVCTRL_CMPEO0;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Enabling RTC.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE; // Enable RTC.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  // Enable RTC interrupt in controller at highest priority.
  NVIC_SetPriority(RTC_IRQn, 0x00);
  NVIC_EnableIRQ(RTC_IRQn);

  DEBUG_PRINTLN("[RtcSync]: Enable continuous synchronization.");
  RTC->MODE0.READREQ.reg |=
      RTC_READREQ_RREQ | RTC_READREQ_RCONT | 0x0010; // Continuous reading
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
}

void RTC_Handler() {
  DEBUG_PRINTLN("[RtcSync]: RTC_Handler.");
  if (RTC->MODE0.INTFLAG.bit.CMP0 && RTC->MODE0.INTFLAG.bit.OVF) {
    DEBUG_PRINTLN("[RtcSync]: Increment seconds.");
    RtcSync::getInstance().incrementSecs();
    DEBUG_PRINTLN(RtcSync::getInstance().getSecs());
    // Clear flags.
    RTC->MODE0.INTFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTFLAG.bit.OVF = 1;
  }
}

void RtcSync::setComp0(const uint32_t comp_0) const {
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  RTC->MODE0.COMP[0].reg = comp_0;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  RTC->MODE0.READREQ.reg |=
      RTC_READREQ_RREQ | RTC_READREQ_RCONT | 0x0010; // Continuous reading
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
}
