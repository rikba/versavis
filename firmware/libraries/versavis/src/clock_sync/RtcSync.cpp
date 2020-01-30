#include "RtcSync.h"

//#include <RTClib.h>

#include "helper.h"
#include "versavis_configuration.h"

RtcSync::RtcSync() : rtc_pub_("/versavis/gnss_sync/rtc", &rtc_msg_) {}

void RtcSync::setup(ros::NodeHandle *nh, Uart *uart,
                    const uint32_t baud_rate /*= 115200*/) {
  setupRos(nh);
  setupSerial(uart, baud_rate);
  setupCounter();
}

void RtcSync::setupRos(ros::NodeHandle *nh) {
  nh_ = nh;
#ifndef DEBUG
  if (nh_) {
    nh_->advertise(rtc_pub_);
    nh_->spinOnce();
  }
#endif
}

void RtcSync::setupSerial(Uart *uart, const uint32_t baud_rate) {
  DEBUG_PRINT("[RtcSync]: Setup serial connection with baud rate ");
  DEBUG_PRINTLN(baud_rate);
  uart_ = uart;
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

void RtcSync::setupCounter() const {
  DEBUG_PRINT("[RtcSync]: Setup PPS periodic counter.");
  setupPort();
  setupGenericClock5();
  setupEic();
  setupRTC();
}

void RtcSync::setupPort() const {
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Enable PORTs.

  DEBUG_PRINTLN("[RtcSync]: Configuring PPS as PA11/EXTINT[11]");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 11); // Set pin PA11 pin as input
  PORT->Group[PORTA].PMUX[11 >> 1].reg |=
      PORT_PMUX_PMUXO_A; // Connect PA pin to peripheral A (EXTINT[11])
  PORT->Group[PORTA].PINCFG[11].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[11].reg |= PORT_PINCFG_INEN; // Enable input
}

void RtcSync::setupGenericClock5() const {
  REG_PM_APBAMASK |= PM_APBAMASK_GCLK; // GCLK APB Clock Enable

  DEBUG_PRINTLN("[RtcSync]: Setting generic clock 5 divider to 4.");
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(4) | GCLK_GENDIV_ID(5);
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[RtcSync]: Configuring GENCTRL register.");
  GCLK->GENCTRL.reg = GCLK_GENCTRL_DIVSEL | GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(5) |
                      GCLK_GENCTRL_RUNSTDBY;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN(
      "[RtcSync]: Enabling generic clock 5 for RTC and edge detection.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK5 | GCLK_CLKCTRL_ID_RTC;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
}

void RtcSync::setupEic() const {
  // REG_PM_APBAMASK |= PM_APBAMASK_EIC; // EIC enable.
  //
  // DEBUG_PRINTLN("[RtcSync]: Configuring EIC on EXTINT11.");
  // // Enable event from pin on external interrupt 11 (EXTINT11)
  // EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO11;
  // // Set event on rise edge of signal
  // EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE3_RISE;
  // EIC->CTRL.reg |= EIC_CTRL_ENABLE; // Enable EIC peripheral
  // while (EIC->STATUS.bit.SYNCBUSY) {
  // } // Wait for synchronization
}

void RtcSync::setupRTC() const {
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

  DEBUG_PRINTLN("[RtcSync]: RTC DIV1024.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_PRESCALER_DIV1024;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: RTC MODE0 COUNT32.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_MODE_COUNT32;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Reset counter.");
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  RTC->MODE0.COUNT.reg = 0;
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
  DEBUG_PRINTLN("[RtcSync]: Enabling RTC.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE; // Enable RTC.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Enable continuous synchronization.");
  RTC->MODE0.READREQ.reg |=
      RTC_READREQ_RREQ | RTC_READREQ_RCONT | 0x0010; // Continuous reading
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
}
