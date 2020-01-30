#include "RtcSync.h"

#include <RTClib.h>

#include "helper.h"
#include "versavis_configuration.h"

RtcSync::RtcSync() : rtc_pub_("/versavis/gnss_sync/rtc", &rtc_msg_) {}

void RtcSync::setup(ros::NodeHandle *nh) {
  setupRos(nh);
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

void RtcSync::setupNmeaSerial(Uart *uart,
                              const uint32_t baud_rate /*= 115200*/) {
  DEBUG_PRINT("[RtcSync]: Setup serial connection with baud rate ");
  DEBUG_PRINTLN(baud_rate);
  uart_ = uart;
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

void RtcSync::setupCounter() const {
  DEBUG_PRINT("[RtcSync]: Setup PPS periodic counter.");
  setupGenericClock5();
  setupRTC();
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

  DEBUG_PRINTLN("[RtcSync]: Enabling generic clock 5 for RTC.");
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK5 | GCLK_CLKCTRL_ID_RTC;
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
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

bool RtcSync::syncGnss() {
  bool received_time = false;
  // Clear UART buffer on first call. There may still be old data.
  if (clear_uart_) {
    while (uart_ && uart_->available()) {
      uart_->read();
    }
    clear_uart_ = false;
  } else { // Read most current NMEA absolute time.
    while (uart_ && uart_->available()) {
      auto result = nmea_parser_.parseChar(uart_->read());
      if (result != NmeaParser::SentenceType::kGpZda)
        continue;
      received_time = (nmea_parser_.getGpZdaMessage().hundreths == 0);
    }
  }

  // Update RTC time.
  if (received_time) {
    DEBUG_PRINTLN(nmea_parser_.getGpZdaMessage().str);
    DateTime date_time(nmea_parser_.getGpZdaMessage().year,
                       nmea_parser_.getGpZdaMessage().month,
                       nmea_parser_.getGpZdaMessage().day,
                       nmea_parser_.getGpZdaMessage().hour,
                       nmea_parser_.getGpZdaMessage().minute,
                       nmea_parser_.getGpZdaMessage().second);

    // See about synchronization.
    // https://community.atmel.com/forum/samd21-rtc-mode0-count-value-freezes-rcont-rreq-set?skey=rtc_readreq_rcont
    while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
    }
    RTC->MODE0.COUNT.reg = date_time.unixtime();
    while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
    }
    RTC->MODE0.READREQ.reg |=
        RTC_READREQ_RREQ | RTC_READREQ_RCONT | 0x0010; // Continuous reading
    while (RTC->MODE0.STATUS.bit.SYNCBUSY) {
    }
  }

  return received_time;
}
