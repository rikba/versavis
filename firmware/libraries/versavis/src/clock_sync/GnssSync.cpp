#include "GnssSync.h"

#include <RTClib.h>
#include <ros/time.h>

#include "helper.h"
#include "nmea_parser/NmeaParser.h"
#include "versavis_configuration.h"

#ifdef GNSS_SYNC_GCLKIN_10MHZ
const uint32_t kClkFreqHz = 10e6;
#elif defined GNSS_SYNC_DFLL48M
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
  setupSerial(uart, baud_rate);
}

void GnssSync::resetFilterState() {
  // Initialize filter state.
  filter_state_.stamp.data = ros::Time(0, 0);
#ifdef GNSS_SYNC_GCLKIN_10MHZ
  const uint16_t jitter = 1.0;        // Ticks jitter per second.
  const float freq_stability = 0.014; // PPM / delta deg C
#elif defined GNSS_SYNC_DFLL48M
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
  if (nh_) {
    nh_->advertise(filter_state_pub_);
    nh_->spinOnce();
  }
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
                           const double ticks_to_nanoseconds, int32_t ticks,
                           uint32_t *sec, uint32_t *nsec) {

  if (sec) {
    *sec = filter_state.t_nmea + filter_state.pps_cnt;
    if (ticks < 0) {
      *sec -= 1;
      ticks = kClkFreqHz + ticks;
    }
  }
  if (nsec) {
    *nsec = ticks_to_nanoseconds * static_cast<double>(ticks);
  }
  ros::normalizeSecNSec(*sec, *nsec);
}

void GnssSync::computeTime(const versavis::ExtClkFilterState &filter_state,
                           const double ticks_to_nanoseconds, int32_t ticks,
                           ros::Time *time) {
  uint32_t sec, nsec;
  computeTime(filter_state, ticks_to_nanoseconds, ticks, &sec, &nsec);
  if (time)
    *time = ros::Time(sec, nsec);
}

ros::Time GnssSync::getTimeNow() {
  // TODO(rikba): Better timing accuracy could be archieved by configuring an
  // event capture.

  // In order to read the COUNT we need to sync first.
  // https://forum.arduino.cc/index.php?topic=396804.30
  int32_t count = TCC0->COUNT.bit.COUNT;
#ifdef GNSS_SYNC_GCLKIN_10MHZ
  // Compensate for interrupt function call delay.
  count -= 19;
#endif
  ros::Time t(0, 0);
  if (!reset_time_) { // Only set time once absolute time is set.
    computeTime(filter_state_, ticks_to_nanoseconds_, count, &t);
  }
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

// Setup the port register.
void GnssSync::setupPort() {
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.

// Setup the external clock if necessary.
#ifdef GNSS_SYNC_GCLKIN_10MHZ
  DEBUG_PRINTLN("[GnssSync]: Configuring PA10/GCLK_IO[4] as input.");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 10); // Set pin PA10 pin as input
  PORT->Group[PORTA].PMUX[10 >> 1].reg |=
      PORT_PMUX_PMUXE_H; // Connect PA10 pin to peripheral H (GCLK_IO[4])
  PORT->Group[PORTA].PINCFG[10].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[10].reg |= PORT_PINCFG_INEN; // Enable input
#endif

  DEBUG_PRINTLN("[GnssSync]: Configuring PPS input on SAMD PA 11");
  PORT->Group[PORTA].DIRCLR.reg =
      PORT_DIRCLR_DIRCLR(1 << 11); // Set pin PA11 pin as input
  PORT->Group[PORTA].PMUX[11 >> 1].reg |=
      PORT_PMUX_PMUXO_A; // Connect PA pin to peripheral A (EXTINT[11])
  PORT->Group[PORTA].PINCFG[11].reg |=
      PORT_PINCFG_PMUXEN; // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[11].reg |= PORT_PINCFG_INEN; // Enable input
}

void GnssSync::setupGenericClock4() {
  REG_PM_APBAMASK |= PM_APBAMASK_GCLK; // GCLK APB Clock Enable

  // Select clock source.
#ifdef GNSS_SYNC_GCLKIN_10MHZ
  DEBUG_PRINTLN("[GnssSync]: Configuring GENCTRL register to route GCLKIN to "
                "generic clock 4.");
  REG_GCLK_GENCTRL =
      GCLK_GENCTRL_GENEN |      // Enable clock.
      GCLK_GENCTRL_SRC_GCLKIN | // Set to external 10MHz oscillator
      GCLK_GENCTRL_ID(4);       // Set clock source to GCLK4
#else
  DEBUG_PRINTLN("[GnssSync]: Configuring GENCTRL register to route XOSC32K to "
                "generic clock 4.");
  REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |       // Enable clock.
                     GCLK_GENCTRL_SRC_XOSC32K | // Set to internal 32kHz osci
                     GCLK_GENCTRL_ID(4);        // Set clock source to GCLK4
#endif
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  // Route clock to TCC0.
  DEBUG_PRINTLN("[GnssSync]: Enabling generic clock for TCC0/TCC1");
  REG_GCLK_CLKCTRL =
      GCLK_CLKCTRL_CLKEN |       // Enable the generic clock...
      GCLK_CLKCTRL_GEN_GCLK4 |   // ....on GCLK4...
      GCLK_CLKCTRL_ID_TCC0_TCC1; // ... to feed the GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  // Activate edge detection.
  DEBUG_PRINTLN("[GnssSync]: Enabling generic clock 4 for edge detection.");
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |     // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK4 | // ....on GCLK4...
                     GCLK_CLKCTRL_ID_EIC;     // ... to detect edges
  while (GCLK->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
}

void GnssSync::setupEic() {
  REG_PM_APBAMASK |= PM_APBAMASK_EIC; // EIC enable.

  // Receive PPS interrupt on PA11.
  DEBUG_PRINTLN("[GnssSync]: Configuring EIC on EXTINT11.");
  // Enable event from pin on external interrupt 11 (EXTINT11)
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO11;
  // Set event on rise edge of signal
  REG_EIC_CONFIG1 |= EIC_CONFIG_SENSE3_RISE;
  REG_EIC_CTRL |= EIC_CTRL_ENABLE; // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
}

void GnssSync::setupEvsys() {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS; // Switch on the event system peripheral

  // Attach the event user (receiver) to channel n=0 (n + 1) (arbitrary)
  // Set the event user (receiver) to timer TCC0, Event 1
  DEBUG_PRINTLN("[GnssSync]: Configuring EVSYS such that TCC0 is event user.");
  REG_EVSYS_USER =
      EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1);

  DEBUG_PRINTLN("[GnssSync]: Configuring Event Channel.");
  REG_EVSYS_CHANNEL =
      EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT | // No event output edge detection
      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |    // Set event path as asynchronous
      EVSYS_CHANNEL_EVGEN(
          EVSYS_ID_GEN_EIC_EXTINT_11) | // Set event generator (sender) as
                                        // external interrupt 11
      EVSYS_CHANNEL_CHANNEL(0); // Attach the generator (sender) to channel 1
  while (EVSYS->CHSTATUS.bit.CHBUSY0) {
  }
}

void GnssSync::setupTCC0() {
  REG_PM_APBCMASK |= PM_APBCMASK_TCC0; // Enable TCC0 Bus clock

  DEBUG_PRINTLN("[GnssSync]: Disabling TCC0.");
  TCC0->CTRLA.bit.ENABLE = 0; // Disable TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[GnssSync]: Setup TCC0 CTRLA.");
  TCC0->CTRLA.reg |=
      TCC_CTRLA_PRESCALER_DIV1 | // Set prescaler to 1, 10MHz/1 = 10MHz
      TCC_CTRLA_CPTEN0;          // Capture channel 0 event (period).
  TCC0->PER.reg = 0xFFFFFF;      // 24 Bit period.
  while (TCC0->SYNCBUSY.bit.PER) {
  }

  DEBUG_PRINTLN("[GnssSync]: Enabling TCC0 EVCTRL.");
  TCC0->EVCTRL.reg |= TCC_EVCTRL_TCEI1 |     // Enable input event 1
                      TCC_EVCTRL_EVACT1_PPW; // Period capture in CC0

  DEBUG_PRINTLN("[GnssSync]: Enabling interrupts on channel 0 events.");
  TCC0->INTENSET.bit.MC0 = 1; // Enable TCC0 interrupts
  DEBUG_PRINTLN("[GnssSync]: Clearing interrupt flags.");
  TCC0->INTFLAG.bit.MC0 = 1; // Clear the interrupt flags

  // Set the Nested Vector Interrupt Controller
  DEBUG_PRINTLN("[GnssSync]: Enabling NVIC.");
  // (NVIC) priority for TCC0 to 0 (highest)
  NVIC_SetPriority(TCC0_IRQn, 0);
  // Connect TCC0 to Nested Vector Interrupt Controller (NVIC)
  NVIC_EnableIRQ(TCC0_IRQn);

  DEBUG_PRINTLN("[GnssSync]: Enable TCC0.");
  TCC0->CTRLA.bit.ENABLE = 1; // Enable TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE) {
  }
}

void GnssSync::setupCounter() {
  DEBUG_PRINT("[GnssSync]: Setup PPS period counter.");
  setupPort();
  setupGenericClock4();
  setupEic();
  setupEvsys();
  setupTCC0();
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

  // In order to read the COUNT we need to sync first.
  // https://forum.arduino.cc/index.php?topic=396804.30
  while (TCC0->SYNCBUSY.bit.CTRLB) {
  }
  TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
  while (TCC0->SYNCBUSY.bit.CTRLB) {
  }
  while (TCC0->SYNCBUSY.bit.COUNT) {
  }
  float duration_ns = float(TCC0->COUNT.bit.COUNT) * ticks_to_nanoseconds_;
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

// Interrupt Service Routine (ISR) for timer TCC0
void TCC0_Handler() {
  if (TCC0->INTFLAG.bit.MC0) {
    // Read period length. Interrupt flag is cleared automatically.
    GnssSync::getInstance().measureTicksPerSecond(TCC0->CC[0].bit.CC);
    GnssSync::getInstance().incrementPPS();

    DEBUG_PRINT("[GnssSync]: Received PPS signal: ");
    DEBUG_PRINTLN(GnssSync::getInstance().getFilterState().pps_cnt + 1);
  }
  // TODO(rikba): catch and manage overflow
}
