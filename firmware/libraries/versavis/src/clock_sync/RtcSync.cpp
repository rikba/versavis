#include "RtcSync.h"

#include <ros/time.h>

#include "helper.h"
#include "versavis_configuration.h"

RtcSync::RtcSync() : rtc_pub_("/versavis/gnss_sync/rtc", &rtc_msg_) {
  setupPort();
  setupGenericClock4();
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

void RtcSync::setupGenericClock4() const {
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

  // Setup all other timer with the same clock.
  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock 4 for TCC0/TCC1.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock 4 for TCC2/TC3.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC2_TC3;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock 4 for TC4/TC5.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TC4_TC5;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
}

void RtcSync::setupEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS; // Enable EVSYS.

  DEBUG_PRINTLN("[RtcSync]: Connect all timers to this event on channel 0.");
  // TCC0
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0); // Retrigger

  // TCC1
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_0); // Capture

  // TCC2
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC2_EV_0); // Retrigger

  // TC3
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU); // Retrigger

  // TC4
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU); // Retrigger

  // TC5
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TC5_EVU); // Retrigger

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

ros::Duration RtcSync::getDuration(const uint32_t ticks) {
  uint32_t secs = 0;
  uint32_t nsecs = ticks * ns_per_tick_;
  ros::normalizeSecNSec(secs, nsecs);
  return ros::Duration(secs, nsecs);
}

ros::Duration RtcSync::getDuration(const uint32_t ticks, uint8_t prescaler) {
  return getDuration(ticks * prescaler);
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
