#include "RtcSync.h"

#include <ros/time.h>

#include "helper.h"
#include "versavis_configuration.h"

RtcSync::RtcSync() {
  setupPort();
  setupGenericClock4();
  setupEvsys();
  setupRtc();
}

void RtcSync::setupRos(ros::NodeHandle *nh, const char *topic) {
  if (nh) {
    // Create static ROS objects.
    static std_msgs::Time msg;
    static ros::Publisher pub(topic, &msg);

    // Set member variables.
    rtc_msg_ = &msg;
    rtc_pub_ = &pub;

    // Advertise topic
    nh->advertise(pub);
  }
}

void RtcSync::publish() {
  if (rtc_pub_ && rtc_msg_ && has_stamp_) {
    rtc_msg_->data = ros::Time(secs_, 0);
    rtc_pub_->publish(rtc_msg_);
    has_stamp_ = false;
  }
}

void RtcSync::setupPort() const {
  // Setup the external clock if necessary.
#ifdef RTC_GCLKIN_10MHZ
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.

  DEBUG_PRINTLN("[GnssSync]: Configuring PA10 as GCLK_IO[4].");
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
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_GCLKIN | GCLK_GENCTRL_ID(4);
#else
  DEBUG_PRINTLN("[RtcSync]: Using XOSC32K.");
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(4);
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
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_0); // Retrigger

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
  // Makes sure that seconds are updated before anyone is accessing it.
  NVIC_SetPriority(RTC_IRQn, 0);
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

ros::Time RtcSync::computeTime(const uint32_t secs, const uint32_t ticks,
                               uint16_t prescaler) const {
  ros::Time time(secs, ticks * prescaler * ns_per_tick_);
  ros::normalizeSecNSec(time.sec, time.nsec);
  return time;
}

ros::Time RtcSync::computeTime(const uint32_t ticks, uint16_t prescaler) const {
  return computeTime(secs_, ticks, prescaler);
}

void RtcSync::computePwm(const uint16_t rate_hz, const uint32_t pulse_us,
                         const uint16_t prescaler, uint32_t *top,
                         uint32_t *duty_cycle) const {
  if (top) {
    *top = clock_freq_ / prescaler / rate_hz - 1;
  }

  if (duty_cycle) {
    uint32_t mega_pulses = pulse_us * clock_freq_;
    *duty_cycle = mega_pulses / 1e6 +
                  (mega_pulses % static_cast<uint32_t>(1e6) != 0); // Ceil.
  }
}
void RtcSync::computeFrq(const uint16_t rate_hz, const uint16_t prescaler,
                         uint32_t *top) const {
  if (top) {
    *top = (clock_freq_ / prescaler / rate_hz) / 2 - 1;
  }
}

uint8_t RtcSync::findMinPrescalerPwm(const uint16_t rate_hz,
                                     const uint32_t counter_max) const {
  bool found_prescaler = false;
  uint8_t prescaler = 0;
  for (; prescaler < sizeof(kPrescalers) / sizeof(kPrescalers[0]);
       ++prescaler) {
    const uint32_t kRequiredTicks =
        clock_freq_ / kPrescalers[prescaler] / static_cast<uint32_t>(rate_hz);
    found_prescaler = (kRequiredTicks < counter_max);
    if (found_prescaler) {
      break;
    }
  }

  if (!found_prescaler) {
    error("NO_PRESCALER (RtcSync.cpp): cannot find suitable prescaler.", 201);
  } else {
    DEBUG_PRINT("[RtcSync]: Found optimal prescaler: ");
    DEBUG_PRINTLN(kPrescalers[prescaler]);
  }

  return prescaler;
}

ros::Time RtcSync::getTimeNow() const {
  // TODO(rikba): Find a non blocking alternative.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  ros::Time now(secs_, RTC->MODE0.COUNT.reg * ns_per_tick_);
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  ros::normalizeSecNSec(now.sec, now.nsec);

  return now;
}

uint8_t RtcSync::findMinPrescalerFrq(const uint16_t rate_hz,
                                     const uint32_t counter_max) const {
  return findMinPrescalerPwm(2 * rate_hz, counter_max);
}

void RTC_Handler() {
  if (RTC->MODE0.INTFLAG.bit.CMP0 && RTC->MODE0.INTFLAG.bit.OVF) {
    RtcSync::getInstance().incrementSecs();
    // Clear flags.
    RTC->MODE0.INTFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTFLAG.bit.OVF = 1;
  }
}
