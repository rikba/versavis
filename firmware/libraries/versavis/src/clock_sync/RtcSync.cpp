#include "RtcSync.h"

#include <RTClib.h>

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
  setupRTC();
}

void RtcSync::setupPort() const {
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Enable PORTs.

  DEBUG_PRINTLN("[RtcSync]: Configuring PPS as PA11/GCLK_IO[5]");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 11); // Set pin PA11 pin as input
  PORT->Group[PORTA].PMUX[11 >> 1].reg |=
      PORT_PMUX_PMUXO_H; // Connect PA pin to peripheral H (GCLK_IO[5])
  PORT->Group[PORTA].PINCFG[11].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[11].reg |= PORT_PINCFG_INEN; // Enable input
}

void RtcSync::setupGenericClock5() const {
  REG_PM_APBAMASK |= PM_APBAMASK_GCLK; // GCLK APB Clock Enable
  REG_PM_APBAMASK |= PM_APBAMASK_RTC;  // Enable RTC.

  // Select clock source.
  DEBUG_PRINTLN("[RtcSync]: Configuring GENCTRL register to route PPS to "
                "generic clock 5.");
  REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |      // Enable clock.
                     GCLK_GENCTRL_SRC_GCLKIN | // Set to GCLK_IO[5]
                     GCLK_GENCTRL_ID(5);       // Set clock source to GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  // Route clock to TCC0.
  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock for RTC");
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |     // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK5 | // ....on GCLK5...
                     GCLK_CLKCTRL_ID_RTC;     // ... to feed the GCLK5 to RTC
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
}

void RtcSync::setupRTC() const {
  REG_PM_APBAMASK |= PM_APBAMASK_RTC; // Enable RTC power manager.

  DEBUG_PRINTLN("[RtcSync]: Disabling RTC.");
  RTC->MODE0.CTRL.bit.ENABLE = 0; // Disable RTC.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Set 32bit counter.");
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_PRESCALER_DIV1 | // No prescaling.
                         RTC_MODE0_CTRL_MODE_COUNT32;    // 32 Bit counter.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Enable continuous synchronization.");
  RTC->MODE0.READREQ.reg |= RTC_READREQ_RCONT; // Continuous reading
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[RtcSync]: Enabling RTC.");
  RTC->MODE0.CTRL.bit.ENABLE = 1; // Enable RTC.
  while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
  }
}
