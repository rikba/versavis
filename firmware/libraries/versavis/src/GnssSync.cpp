#include "GnssSync.h"

#include <RTClib.h>
#include <ros/time.h>

#include "helper.h"
#include "nmea_parser/NmeaParser.h"
#include "versavis_configuration.h"

GnssSync::GnssSync()
    : filter_state_pub_("/versavis/gnss_sync/filter_state", &filter_state_) {}

void GnssSync::setup(ros::NodeHandle *nh, Uart *uart,
                     const uint32_t baud_rate /*= 115200*/) {
  reset();
  setupRos(nh);
  setupSerial(uart, baud_rate);
  setupCounter();
}

void GnssSync::resetFilterState() {
  // Initialize filter state.
  filter_state_.stamp.data = ros::Time(0, 0);
#ifdef USE_GCLKIN_10MHZ
  filter_state_.x = 10000000.0;
  filter_state_.R = 100.0;
  filter_state_.Q = 1.0;
#elif defined USE_DFLL48M
  filter_state_.x = 48000000.0;
  filter_state_.R = 10000.0;
  filter_state_.Q = 100.0;
#else
  filter_state_.x = 32768.0;
  filter_state_.R = 100.0;
  filter_state_.Q = 1.0;
#endif
  filter_state_.P = filter_state_.R;
  filter_state_.z = 0;
  filter_state_.pps_cnt = 0;
  filter_state_.pps_cnt_prev = 0;
  filter_state_.t_nmea = 0;
  filter_state_.x_nspt = 1000000000.0 / filter_state_.x;
}

void GnssSync::setupRos(ros::NodeHandle *nh) {
  nh_ = nh;
#ifndef DEBUG
  if (nh_)
    nh_->advertise(filter_state_pub_);
#endif
}

void GnssSync::setMeasurementNoise(const double R_tps) {
  filter_state_.R = R_tps;
}

void GnssSync::setProcessNoise(const double Q_tps) { filter_state_.Q = Q_tps; }

void GnssSync::update() {
  // Fix volatile variables.
  filter_state_.pps_cnt = pps_cnt_;
  filter_state_.z = z_;

  if (filter_state_.pps_cnt < 2)
    return;

  if (reset_time_) {
    waitForNmea();
    return;
  }

  updateTps();
}

void GnssSync::updateTps() {
  // Update Kalman filter to estimate ticks per second.
  if (filter_state_.pps_cnt <= filter_state_.pps_cnt_prev)
    return;
  // Prediction.
  double dt = filter_state_.pps_cnt - filter_state_.pps_cnt_prev;
  filter_state_.P = filter_state_.P + dt * filter_state_.Q;
  // Measurement.
  double y = static_cast<double>(filter_state_.z) - filter_state_.x;
  double S_inv = 1.0 / (filter_state_.P + filter_state_.R);
  double K = filter_state_.P * S_inv;

  filter_state_.x += K * y;
  filter_state_.P = (1.0 - K) * filter_state_.P;
  filter_state_.x_nspt = 1000000000.0 / filter_state_.x;
  computeTime(filter_state_, 0, &filter_state_.stamp.data);

  DEBUG_PRINT("[GnssSync]: Updated ticks per second ");
  DEBUG_PRINT("x: ");
  DEBUG_PRINT(filter_state_.x);
  DEBUG_PRINT(" P: ");
  DEBUG_PRINTLN(filter_state_.P);

#ifndef DEBUG
  filter_state_pub_.publish(&filter_state_);
#endif
  filter_state_.pps_cnt_prev = filter_state_.pps_cnt;
}

void GnssSync::reset() {
  reset_time_ = true;
  resetFilterState();
}

void GnssSync::computeTime(const versavis::ExtClkFilterState &filter_state,
                           const uint32_t ticks, uint32_t *sec,
                           uint32_t *nsec) {
  if (sec) {
    *sec = filter_state.t_nmea + filter_state.pps_cnt;
  }
  if (nsec) {
    *nsec = double(ticks) * filter_state.x_nspt;
  }
  ros::normalizeSecNSec(*sec, *nsec);
}

void GnssSync::computeTime(const versavis::ExtClkFilterState &filter_state,
                           const uint32_t ticks, ros::Time *time) {
  uint32_t sec, nsec;
  computeTime(filter_state, ticks, &sec, &nsec);
  if (time)
    *time = ros::Time(sec, nsec);
}

void GnssSync::getTimeNow(uint32_t *sec, uint32_t *nsec) {
  computeTime(filter_state_, REG_TC4_COUNT32_COUNT, sec, nsec);
}

// Return time once. Resets valid flag.
bool Timestamp::getTime(uint32_t *sec, uint32_t *nsec) {
  if (!hasTime())
    return false;

  GnssSync::computeTime(filter_state_, ticks_, sec, nsec);
  has_time_ = false;

  return true;
}

void Timestamp::setTime(const versavis::ExtClkFilterState &filter_state,
                        const uint32_t ticks) {
  filter_state_ = filter_state;
  ticks_ = ticks;
  has_time_ = true;
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
  // Make sure pps_cnt_ is at least 200 ms old and at most 800 ms old. Otherwise
  // NMEA signal may have not arrived, yet or is going to be updated already.
  const double kNmeaOffsetNs = 200.0 * 1.0e6;
  const double kLowerLimit = kNmeaOffsetNs;
  const double kUpperLimit = 1e9 - kNmeaOffsetNs;
  double duration_ns = double(REG_TC4_COUNT32_COUNT) * filter_state_.x_nspt;
  if (duration_ns < kLowerLimit || duration_ns > kUpperLimit) {
    DEBUG_PRINT("[GnssSync]: Waiting for PPS count signal to be between ");
    DEBUG_PRINT(kLowerLimit);
    DEBUG_PRINT(" [ns] and ");
    DEBUG_PRINT(kUpperLimit);
    DEBUG_PRINT(" [ns]. Current age: ");
    DEBUG_PRINT(duration_ns);
    DEBUG_PRINTLN(" [ns]");
    return;
  }

  NmeaParser nmea_parser;
  uint32_t t_nmea_pps_cnt = pps_cnt_; // Assign pps signal to time.
  while (uart_ && uart_->available()) {
    auto result = nmea_parser.parseChar(uart_->read());
    if ((result == NmeaParser::SentenceType::kGpZda) &&
        nmea_parser.getGpZdaMessage().hundreths == 0) {
      DEBUG_PRINTLN(nmea_parser.getGpZdaMessage().str);
      reset_time_ = false;
    }
  }
  if (reset_time_)
    return; // Was not able to read time from serial port.

  // Save the time when pps counting started.
  DateTime date_time(
      nmea_parser.getGpZdaMessage().year, nmea_parser.getGpZdaMessage().month,
      nmea_parser.getGpZdaMessage().day, nmea_parser.getGpZdaMessage().hour,
      nmea_parser.getGpZdaMessage().minute,
      nmea_parser.getGpZdaMessage().second);
  filter_state_.t_nmea = date_time.unixtime() - t_nmea_pps_cnt;

#ifdef DEBUG
  DEBUG_PRINTLN("[GnssSync]: Received NMEA time.");
  DEBUG_PRINT("[GnssSync]: Unix time at pps_cnt == 0: ");
  DEBUG_PRINTLN(filter_state_.t_nmea);
#endif
}

// Interrupt Service Routine (ISR) for timer TC4
void TC4_Handler() {
  if (TC4->COUNT32.INTFLAG.bit.MC0) {
    GnssSync::getInstance().measureTicksPerSecond(REG_TC4_COUNT32_CC0);
    GnssSync::getInstance().incrementPPS();
    REG_TC4_INTFLAG = TC_INTFLAG_MC0; // Clear the MC0 interrupt flag

    DEBUG_PRINT("[GnssSync]: Received PPS signal: ");
    DEBUG_PRINTLN(GnssSync::getInstance().getFilterState().pps_cnt + 1);
  }
  // TODO(rikba): catch and manage overflow
}

void EIC_Handler() {
  if (!GnssSync::getInstance().valid()) {
    REG_EIC_INTFLAG |= EIC_INTFLAG_EXTINT14; // Clear flag.
    return;
  }

  if (REG_EIC_INTFLAG & EIC_INTFLAG_EXTINT14) {
    GnssSync::getInstance().setTimePa14(REG_TC4_COUNT32_COUNT);
    REG_EIC_INTFLAG |= EIC_INTFLAG_EXTINT14; // Clear flag.
  }
}

bool GnssSync::getTimePa14(uint32_t *sec, uint32_t *nsec) {
  return timestamp_pa14_.getTime(sec, nsec);
}

void GnssSync::setTimePa14(const uint32_t ticks) {
  timestamp_pa14_.setTime(filter_state_, ticks);
}
