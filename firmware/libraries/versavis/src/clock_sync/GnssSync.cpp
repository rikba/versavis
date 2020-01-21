#include "GnssSync.h"

#include <RTClib.h>
#include <ros/time.h>

#include "helper.h"
#include "nmea_parser/NmeaParser.h"
#include "versavis_configuration.h"

#ifdef USE_GCLKIN_10MHZ
const uint32_t kClkFreqHz = 10e6;
#elif defined USE_DFLL48M
const uint32_t kClkFreqHz = 48e6;
#else
const uint32_t kClkFreqHz = 32768;
#endif

const float kH = kClkFreqHz * 1.0e-6; // Converts ppm to ticks per second.
const double kTicksToSeconds = 1.0 / kClkFreqHz;

GnssSync::GnssSync()
    : filter_state_pub_("/versavis/gnss_sync/filter_state", &filter_state_),
      ticks_to_nanoseconds_(1.0e9 / kClkFreqHz) {}

void GnssSync::setup(ros::NodeHandle *nh, Uart *uart,
                     const uint32_t baud_rate /*= 115200*/) {
  reset();
  setupRos(nh);
  timestamp_pa14_ = Timestamp(nh);
  setupSerial(uart, baud_rate);
  setupCounter();
}

void GnssSync::resetFilterState() {
  // Initialize filter state.
  filter_state_.stamp.data = ros::Time(0, 0);
#ifdef USE_GCLKIN_10MHZ
  const uint16_t jitter = 1.0;        // Ticks jitter per second.
  const float freq_stability = 0.014; // PPM / delta deg C
#elif defined USE_DFLL48M
  const uint16_t jitter = 4000.0;
  const float freq_stability = 0.04; // PPM / delta deg C
#else
  const uint16_t jitter = 1.0;       // Jitter per second.
  const float freq_stability = 0.04; // PPM / delta deg C
#endif
#
  // Assume temperature changes 1 deg / minute is sigma.
  filter_state_.Q = pow(freq_stability / 60.0, 2.0);
  // Assume jitter + discretization error + interrupt cycles is sigma.
  const float interrupt_cycles = 20.0 * (kClkFreqHz / CPU_FREQ_HZ);
  const float pps_accuracy = 60.0 * 1.0e-9 * kClkFreqHz;
  filter_state_.R = pow((jitter + interrupt_cycles + pps_accuracy + 1.0), 2.0);
  filter_state_.P = filter_state_.R / kH / kH;
  filter_state_.x = 0;
  filter_state_.z = 0;
  filter_state_.pps_cnt = 0;
  filter_state_.pps_cnt_prev = 0;
  filter_state_.t_nmea = 0;
}

void GnssSync::setupRos(ros::NodeHandle *nh) {
  nh_ = nh;
#ifndef DEBUG
  if (nh_)
    nh_->advertise(filter_state_pub_);
#endif
}

void GnssSync::setMeasurementNoise(const float R_tps) {
  filter_state_.R = R_tps;
}

void GnssSync::setProcessNoise(const float Q_tps) { filter_state_.Q = Q_tps; }

void GnssSync::update() {
  // Fix volatile variables.
  filter_state_.pps_cnt = pps_cnt_;
  filter_state_.z = z_ - kClkFreqHz;

  if (filter_state_.pps_cnt < 2) {
    // Do nothing.
  } else if (reset_time_) {
    reset_time_ = !waitForNmea(); // Get absolute time.
  } else {
    updateTps(); // Update Kalman filter.
  }
}

void GnssSync::updateTps() {
  // Update Kalman filter to estimate ticks per second.
  if (filter_state_.pps_cnt <= filter_state_.pps_cnt_prev)
    return;

  // Prediction.
  float dt = filter_state_.pps_cnt - filter_state_.pps_cnt_prev;
  filter_state_.P = filter_state_.P + dt * filter_state_.Q;
  // Measurement.
  float y = filter_state_.z - kH * filter_state_.x;
  float S_inv = 1.0 / (kH * filter_state_.P * kH + filter_state_.R);
  float K = filter_state_.P * kH * S_inv;

  filter_state_.x += K * y;
  filter_state_.P = (1.0 - K * kH) * filter_state_.P;
  ticks_to_nanoseconds_ =
      1.0e9 / (static_cast<double>(kH) * static_cast<double>(filter_state_.x) +
               static_cast<double>(kClkFreqHz));
  computeTime(filter_state_, ticks_to_nanoseconds_, 0,
              &filter_state_.stamp.data);

  DEBUG_PRINT("[GnssSync]: Updated ppm ");
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
  clear_uart_ = true;
  resetFilterState();
}

void GnssSync::computeTime(const versavis::ExtClkFilterState &filter_state,
                           const double ticks_to_nanoseconds,
                           const uint32_t ticks, uint32_t *sec,
                           uint32_t *nsec) {
  if (sec) {
    *sec = filter_state.t_nmea + filter_state.pps_cnt;
  }
  if (nsec) {
    *nsec = ticks_to_nanoseconds * static_cast<double>(ticks);
  }
  ros::normalizeSecNSec(*sec, *nsec);
}

void GnssSync::computeTime(const versavis::ExtClkFilterState &filter_state,
                           const double ticks_to_nanoseconds,
                           const uint32_t ticks, ros::Time *time) {
  uint32_t sec, nsec;
  computeTime(filter_state, ticks_to_nanoseconds, ticks, &sec, &nsec);
  if (time)
    *time = ros::Time(sec, nsec);
}

void GnssSync::getTimeNow(uint32_t *sec, uint32_t *nsec) {
  computeTime(filter_state_, ticks_to_nanoseconds_, REG_TC4_COUNT32_COUNT, sec,
              nsec);
}

ros::Time GnssSync::getTimeNow() {
  ros::Time t;
  computeTime(filter_state_, ticks_to_nanoseconds_, REG_TC4_COUNT32_COUNT, &t);
  return t;
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

bool GnssSync::waitForNmea() {
  // State variables.
  NmeaParser nmea_parser;
  filter_state_.x = filter_state_.z / kH;
  filter_state_.pps_cnt = pps_cnt_;
  filter_state_.pps_cnt_prev = filter_state_.pps_cnt;

  // Make sure pps_cnt_ is at least 200 ms old and at most 800 ms old. Otherwise
  // NMEA signal may have not arrived, yet or is going to be updated.
  const float kNmeaOffsetNs = 200.0 * 1.0e6;
  const float kLowerLimit = kNmeaOffsetNs;
  const float kUpperLimit = 1e9 - kNmeaOffsetNs;
  float duration_ns = float(REG_TC4_COUNT32_COUNT) * ticks_to_nanoseconds_;
  bool uart_arrived = (duration_ns > kLowerLimit);
  uart_arrived &= (duration_ns < kUpperLimit);

#ifdef DEBUG
  if (!uart_arrived) {
    DEBUG_PRINT("[GnssSync]: Waiting for PPS count signal to be between ");
    DEBUG_PRINT(kLowerLimit);
    DEBUG_PRINT(" [ns] and ");
    DEBUG_PRINT(kUpperLimit);
    DEBUG_PRINT(" [ns]. Current age: ");
    DEBUG_PRINT(duration_ns);
    DEBUG_PRINTLN(" [ns]");
  }
#endif

  // Read UART.
  bool received_time = false;
  // Clear UART buffer on first call. There may still be old data in the
  if (clear_uart_ && uart_arrived && uart_) {
    while (uart_->available()) {
      uart_->read();
      clear_uart_ = false;
    }
  }
  // Read most current NMEA absolute time.
  else if (uart_arrived && uart_) {
    while (uart_->available()) {
      auto result = nmea_parser.parseChar(uart_->read());
      if (!clear_uart_ && (result == NmeaParser::SentenceType::kGpZda) &&
          (nmea_parser.getGpZdaMessage().hundreths == 0)) {
        DEBUG_PRINTLN(nmea_parser.getGpZdaMessage().str);
        received_time = true;
      }
    }
  }

  // Update filter absolute time with time when pps counting started.
  if (received_time) {
    DateTime date_time(
        nmea_parser.getGpZdaMessage().year, nmea_parser.getGpZdaMessage().month,
        nmea_parser.getGpZdaMessage().day, nmea_parser.getGpZdaMessage().hour,
        nmea_parser.getGpZdaMessage().minute,
        nmea_parser.getGpZdaMessage().second);
    filter_state_.t_nmea = date_time.unixtime() - filter_state_.pps_cnt;
    computeTime(filter_state_, ticks_to_nanoseconds_, 0,
                &filter_state_.stamp.data);

#ifndef DEBUG
    filter_state_pub_.publish(&filter_state_); // Publish initial filter state.
#endif

    DEBUG_PRINTLN("[GnssSync]: Received NMEA time.");
    DEBUG_PRINT("[GnssSync]: Unix time at pps_cnt == 0: ");
    DEBUG_PRINTLN(filter_state_.t_nmea);
    DEBUG_PRINT("[GnssSync]: Unix time at pps_cnt == ");
    DEBUG_PRINT(filter_state_.pps_cnt);
    DEBUG_PRINT(": ");
    DEBUG_PRINTLN(date_time.unixtime());
  }

  return received_time;
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
  timestamp_pa14_.setTime(filter_state_, ticks_to_nanoseconds_, ticks);
}
