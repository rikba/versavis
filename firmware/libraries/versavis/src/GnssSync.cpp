#include "GnssSync.h"

#include <MicroNMEA.h>
#include <RTClib.h>
#include <ros/time.h>

#include "helper.h"
#include "versavis_configuration.h"

void GnssSync::setup(Uart *uart, const uint32_t baud_rate /*= 115200*/) {
  setupSerial(uart, baud_rate);
  setupCounter();
}

void GnssSync::setTimeoutNmea(const uint8_t timeout_nmea_s) {
  timeout_nmea_s_ = timeout_nmea_s;
}

void GnssSync::setMeasurementNoise(const double R_tps) { R_tps_ = R_tps; }

void GnssSync::setProcessNoise(const double Q_tps) { Q_tps_ = Q_tps; }

void GnssSync::update() {
  if (pps_cnt_ < 2)
    return;

  if (pps_cnt_ != pps_cnt_prev_) {
    updateTps();
    pps_cnt_prev_ = pps_cnt_;
  }

  if (reset_time_) {
    waitForNmea();
  }
}

void GnssSync::updateTps() {
  // Update Kalman filter to estimate ticks per second.
  // Prediction.
  P_tps_ = P_tps_ + static_cast<double>(pps_cnt_ - pps_cnt_prev_) * Q_tps_;
  // Measurement.
  double y = static_cast<double>(tps_meas_) - x_tps_;
  double S_inv = 1.0 / (P_tps_ + R_tps_);
  double K = P_tps_ * S_inv;

  x_tps_ += K * y;
  P_tps_ = (1.0 - K) * P_tps_;
  x_nspt_ = 1000000000.0 / x_tps_;

  DEBUG_PRINT("[GnssSync]: Updated ticks per second ");
  DEBUG_PRINT("x_tps: ");
  DEBUG_PRINT(x_tps_);
  DEBUG_PRINT(" P_tps: ");
  DEBUG_PRINTLN(P_tps_);
}

void GnssSync::reset() {
  reset_time_ = true;
  pps_cnt_ = 0;
}

void GnssSync::computeTime(const uint32_t t_nmea, const uint32_t pps_cnt,
                           const uint32_t ticks, const double nspt,
                           uint32_t *sec, uint32_t *nsec) {
  if (sec) {
    *sec = t_nmea + pps_cnt;
  }
  if (nsec) {
    *nsec = double(ticks) * nspt;
  }
  ros::normalizeSecNSec(*sec, *nsec);
}

void GnssSync::getTimeNow(uint32_t *sec, uint32_t *nsec) {
  computeTime(t_nmea_, pps_cnt_, REG_TC4_COUNT32_COUNT, x_nspt_, sec, nsec);
}

// Return time once. Resets valid flag.
bool Timestamp::getTime(uint32_t *sec, uint32_t *nsec) {
  if (!hasTime())
    return false;

  GnssSync::computeTime(t_nmea_, pps_cnt_, ticks_, x_nspt_, sec, nsec);
  valid_ = false;

  return true;
}

void Timestamp::setTime(const uint32_t t_nmea, const uint32_t pps_cnt,
                        const uint32_t ticks, const double x_nspt) {
  t_nmea_ = t_nmea;
  pps_cnt_ = pps_cnt;
  ticks_ = ticks;
  x_nspt_ = x_nspt;
  valid_ = true;
}

void GnssSync::setupSerial(Uart *uart, const uint32_t baud_rate) {
  DEBUG_PRINT("[GnssSync]: Setup serial connection with baud rate ");
  DEBUG_PRINTLN(baud_rate);
  uart_ = uart;
  if (uart_) {
    uart_->begin(baud_rate);
  }
}

void GnssSync::setupCounter() {
  DEBUG_PRINT("[GnssSync]: Setup PPS interrupt and counter.");

  DEBUG_PRINTLN("[GnssSync]: Enabling system peripheral.");
  REG_PM_APBAMASK |= PM_APBAMASK_GCLK;  // GCLK APB Clock Enable
  REG_PM_APBBMASK |= PM_APBBMASK_PORT;  // Port ABP Clock Enable.
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS; // Switch on the event system peripheral
  REG_PM_APBCMASK |= PM_APBCMASK_TC4;   // Enable TC4 Bus clock
  REG_PM_APBAMASK |= PM_APBAMASK_EIC;   // EIC enable.

#ifdef USE_GCLKIN_10MHZ
  DEBUG_PRINTLN("[GnssSync]: Configuring PA10/GCLK_IO[4] as input.");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 10); // Set pin PA10 pin as input
  PORT->Group[PORTA].PMUX[10 >> 1].reg |=
      PORT_PMUX_PMUXE_H; // Connect PA10 pin to peripheral H (GCLK_IO[4])
  PORT->Group[PORTA].PINCFG[10].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[10].reg |= PORT_PINCFG_INEN; // Enable input

  DEBUG_PRINTLN("[GnssSync]: Configuring GENCTRL register to route GCLKIN to "
                "generic clock 4.");
  REG_GCLK_GENCTRL =
      GCLK_GENCTRL_GENEN |      // Enable clock.
      GCLK_GENCTRL_SRC_GCLKIN | // Set to external 10MHz oscillator
      GCLK_GENCTRL_ID(4);       // Set clock source to GCLK4
#elif defined USE_DFLL48M
  DEBUG_PRINTLN("[GnssSync]: Configuring GENCTRL register to route DFLL48M to "
                "generic clock 4.");
  REG_GCLK_GENCTRL =
      GCLK_GENCTRL_GENEN |       // Enable clock.
      GCLK_GENCTRL_SRC_DFLL48M | // Set to internal PLL 48MHz clock
      GCLK_GENCTRL_ID(4);        // Set clock source to GCLK4
#else
  DEBUG_PRINTLN("[GnssSync]: Configuring GENCTRL register to route XOSC32K to "
                "generic clock 4.");
  REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |       // Enable clock.
                     GCLK_GENCTRL_SRC_XOSC32K | // Set to internal 32kHz osci
                     GCLK_GENCTRL_ID(4);        // Set clock source to GCLK4
#endif
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[GnssSync]: Enabling generic clock for TC4/TC5");
  REG_GCLK_CLKCTRL =
      GCLK_CLKCTRL_CLKEN |     // Enable the generic clock...
      GCLK_CLKCTRL_GEN_GCLK4 | // ....on GCLK4...
      GCLK_CLKCTRL_ID_TC4_TC5; // ... to feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[GnssSync]: Configuring input on SAMD PA 11");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 11); // Set pin PA11 pin as input
  PORT->Group[PORTA].PMUX[11 >> 1].reg |=
      PORT_PMUX_PMUXO_A; // Connect PA pin to peripheral A (EXTINT[11])
  PORT->Group[PORTA].PINCFG[11].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[11].reg |= PORT_PINCFG_INEN; // Enable input

  DEBUG_PRINTLN("[GnssSync]: Enabling generic clock 4 for edge detection.");
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |     // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK4 | // ....on GCLK4...
                     GCLK_CLKCTRL_ID_EIC;     // ... to detect edges
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[GnssSync]: Configuring EIC on EXTINT11.");
  // Enable event from pin on external interrupt 11 (EXTINT11)
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO11;
  // Set event on rise edge of signal
  REG_EIC_CONFIG1 |= EIC_CONFIG_SENSE3_RISE;
  REG_EIC_CTRL |= EIC_CTRL_ENABLE; // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[GnssSync]: Configuring EVSYS such that TC4 is event user.");
  // Attach the event user (receiver) to channel n=0 (n + 1)
  // Set the event user (receiver) to timer TC4
  REG_EVSYS_USER =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);

  DEBUG_PRINTLN("[GnssSync]: Configuring Event Channel.");
  REG_EVSYS_CHANNEL =
      EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | // No event output edge detection
      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |    // Set event path as asynchronous
      EVSYS_CHANNEL_EVGEN(
          EVSYS_ID_GEN_EIC_EXTINT_11) | // Set event generator (sender) as
                                        // external interrupt 11
      EVSYS_CHANNEL_CHANNEL(0); // Attach the generator (sender) to channel 0

  DEBUG_PRINTLN("[GnssSync]: Disabling TC4.");
  REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE; // Disable TC4

  DEBUG_PRINTLN("[GnssSync]: Setting TC4 to 32 Bit.");
  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT32; // Set the counter to 32-bit mode
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[GnssSync]: Enabling TC4 input event.");
  REG_TC4_EVCTRL |= TC_EVCTRL_TCEI |         // Enable the TC4 event input
                    TC_EVCTRL_EVACT_PPW_Val; // Period capture in CC0

  DEBUG_PRINTLN("[GnssSync]: Capturing channel 0 event.");
  REG_TC4_CTRLC |= TC_CTRLC_CPTEN0; // Capture channel 0 event.

  DEBUG_PRINTLN("[GnssSync]: Enabling NVIC.");
  // Set the Nested Vector Interrupt Controller
  // (NVIC) priority for TC4 to 0 (highest)
  NVIC_SetPriority(TC4_IRQn, 0);
  // Connect TC4 to Nested Vector Interrupt Controller (NVIC)
  NVIC_EnableIRQ(TC4_IRQn);

  DEBUG_PRINTLN("[GnssSync]: Clearing interrupt flags.");
  REG_TC4_INTFLAG |= TC_INTFLAG_MC0; // Clear the interrupt flags
  DEBUG_PRINTLN("[GnssSync]: Enabling interrupts on channel 0 events.");
  REG_TC4_INTENSET = TC_INTENSET_MC0; // Enable TC4 interrupts

  DEBUG_PRINTLN("[GnssSync]: Disabling prescaler and enable TC4.");
  REG_TC4_CTRLA |=
      TC_CTRLA_PRESCALER_DIV1 | // Set prescaler to 1, 10MHz/1 = 10MHz
      TC_CTRLA_ENABLE;          // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  setupInterruptPa14();
}

void GnssSync::setupInterruptPa14() {

  DEBUG_PRINTLN("[GnssSync]: Configuring input on SAMD PA 14");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 14); // Set pin PA14 pin as input
  PORT->Group[PORTA].PMUX[14 >> 1].reg |=
      PORT_PMUX_PMUXE_A; // Connect PA pin to peripheral A (EXTINT[14])
  PORT->Group[PORTA].PINCFG[14].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[14].reg |= PORT_PINCFG_INEN; // Enable input

  DEBUG_PRINTLN("[GnssSync]: Configuring EIC on EXTINT14.");
  // Enable interrupt
  REG_EIC_INTENCLR |= EIC_INTENCLR_EXTINT14;
  REG_EIC_INTENSET |= EIC_INTENSET_EXTINT14;
  REG_EIC_INTFLAG |= EIC_INTFLAG_EXTINT14;   // Clear flag
  REG_EIC_CONFIG1 |= EIC_CONFIG_SENSE6_RISE; // Rising edge.
  REG_EIC_CTRL |= EIC_CTRL_ENABLE;           // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  NVIC_DisableIRQ(EIC_IRQn);
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_SetPriority(EIC_IRQn, 1);
  NVIC_EnableIRQ(EIC_IRQn);
}

void GnssSync::waitForNmea() {
  reset_time_ = false;
  char nmea_buffer[255];
  MicroNMEA nmea(nmea_buffer, sizeof(nmea_buffer));

  bool time_valid = false;
  uint32_t start_time = millis();
  uint32_t duration_s = 0;
  uint32_t t_nmea_pps_cnt = pps_cnt_; // Assign pps signal to time.

  // Find the unix time that belongs to the last pps pulse.
  DEBUG_PRINTLN("[GnssSync]: Waiting for NMEA absolute time.");
  while (!time_valid && duration_s < timeout_nmea_s_) {
    t_nmea_pps_cnt = pps_cnt_;
    while (uart_ && uart_->available()) {
      nmea.process(Serial.read());
    }
    time_valid = nmea.getYear() != 0 && nmea.getHour() != 99 &&
                 nmea.getHundredths() == 0;
  }

  // Save the time when pps counting started.
  DateTime date_time;
  if (time_valid) {
    date_time = DateTime(nmea.getYear(), nmea.getMonth(), nmea.getDay(),
                         nmea.getHour(), nmea.getMinute(), nmea.getSecond());
  }
  t_nmea_ = date_time.unixtime() - t_nmea_pps_cnt;

#ifdef DEBUG
  if (time_valid) {
    DEBUG_PRINTLN("[GnssSync]: Received NMEA time.");
  }
  DEBUG_PRINT("[GnssSync]: Unix time at pps_cnt == 0: ");
  DEBUG_PRINTLN(t_nmea_);
#endif
}

uint32_t GnssSync::getTnmea() { return t_nmea_; }
uint32_t GnssSync::getPpsCnt() { return pps_cnt_; }
double GnssSync::getNspt() { return x_nspt_; }
uint32_t GnssSync::getTpsMeas() { return tps_meas_; }

// Interrupt Service Routine (ISR) for timer TC4
void TC4_Handler() {
  if (TC4->COUNT32.INTFLAG.bit.MC0) {
    GnssSync::getInstance().measureTicksPerSecond(REG_TC4_COUNT32_CC0);
    GnssSync::getInstance().incrementPPS();
    REG_TC4_INTFLAG = TC_INTFLAG_MC0; // Clear the MC0 interrupt flag

    DEBUG_PRINT("[GnssSync]: Received PPS signal: ");
    DEBUG_PRINT(GnssSync::getInstance().getPpsCnt());
    DEBUG_PRINT(" Ticks: ");
    DEBUG_PRINTLN(GnssSync::getInstance().getTpsMeas());
  }
  // TODO(rikba): catch and manage overflow
}

void EIC_Handler() {
  if (REG_EIC_INTFLAG & EIC_INTFLAG_EXTINT14) {
    GnssSync::getInstance().setTimePa14(REG_TC4_COUNT32_COUNT);
    REG_EIC_INTFLAG |= EIC_INTFLAG_EXTINT14; // Clear flag.
  }
}

bool GnssSync::getTimePa14(uint32_t *sec, uint32_t *nsec) {
  return timestamp_pa14_.getTime(sec, nsec);
}

void GnssSync::setTimePa14(const uint32_t ticks) {
  timestamp_pa14_.setTime(t_nmea_, pps_cnt_, ticks, x_nspt_);
}
