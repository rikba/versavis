#include "clock_sync/ClockEstimator.h"
#include "clock_sync/RtcSync.h"
#include "clock_sync/Tcc0Synced.h"
#include "sensors/ExternalClock.h"

#define DACTOVOLT RTC_CLK_SYNC_U_REF / RTC_CLK_SYNC_DAC_RANGE
#define VOLTTODAC RTC_CLK_SYNC_DAC_RANGE / RTC_CLK_SYNC_U_REF

ExternalClock::ExternalClock(ros::NodeHandle *nh)
    : SensorSynced(nh, &Tcc0Synced::getInstance()) {
  // Setup DAC
  // Connect PA2 pin to peripheral B (VOUT)
  PORT->Group[PORTA].PMUX[2 >> 1].reg |= PORT_PMUX_PMUXE_B;
  // Enable pin peripheral multiplexation
  PORT->Group[PORTA].PINCFG[2].reg |= PORT_PINCFG_PMUXEN;

  while (DAC->STATUS.bit.SYNCBUSY) {
  }
  DAC->CTRLA.bit.SWRST = 1; // Reset and disable DAC
  while (DAC->STATUS.bit.SYNCBUSY) {
  }

  DAC->CTRLB.reg |= DAC_CTRLB_REFSEL_AVCC; // 3.3V reference.
  DAC->CTRLB.reg |= DAC_CTRLB_EOEN;        // Enable voltage output.

  DAC->DATA.reg |= static_cast<uint16_t>(
      VOLTTODAC * RTC_CLK_SYNC_X1); // 1.5V nominal current
  while (DAC->STATUS.bit.SYNCBUSY) {
  }

  DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  while (DAC->STATUS.bit.SYNCBUSY) {
  }

  if (timer_) {
    const bool kInvert = false;
    static_cast<Tcc0Synced *>(timer_)->setupPps(kInvert);
  }
  resetFilter();
}

void ExternalClock::setupRos(const char *topic) {
  // Create static ROS message.
  static versavis::ExtClk clock_msg;
  clock_msg_ = &clock_msg;

  if (nh_) {

    // Create static ROS publisher.
    static ros::Publisher pub(topic, clock_msg_);
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(pub);
  }
}

void ExternalClock::publish() {
  switch (state_) {
  case State::kWaitForPulse: {
    if (timer_ && clock_msg_ &&
        static_cast<TccSynced *>(timer_)->getTimeLastPps(
            &clock_msg_->receive_time, &clock_msg_->pps_cnt)) {
      state_ = State::kWaitForRemoteTime;
    }
    break;
  }
  case State::kWaitForRemoteTime: {
    auto remote_time_status = setRemoteTime();
    if (remote_time_status == RemoteTimeStatus::kReceived) {
      state_ = State::kUpdateFilter;
    } else if (remote_time_status == RemoteTimeStatus::kTimeout) {
      state_ = State::kWaitForPulse;
    }
    break;
  }
  case State::kUpdateFilter: {
    updateFilter();
    controlClock();
    state_ = State::kPublishFilterState;
    break;
  }
  case State::kPublishFilterState: {
    if (publisher_) {
      publisher_->publish(clock_msg_);
    }
    state_ = State::kWaitForPulse;
    break;
  }
  default: {
    state_ = State::kWaitForPulse;
    break;
  }
  }
}

void ExternalClock::updateFilter() {
  if (!clock_msg_)
    return;

  if (last_update_.sec == 0 && last_update_.nsec == 0) {
    // Initialize filter.
    auto offset = clock_msg_->receive_time - clock_msg_->remote_time;
    measured_offset_s_ = offset.toSec();
    clock_msg_->x[0] = toUSec(offset);
    clock_msg_->x[1] = RTC_CLK_SYNC_X1;
    clock_msg_->x[2] = RTC_CLK_SYNC_X2;

    clock_msg_->P[0] = pow(RTC_CLK_SYNC_X0_OFFSET / 3.0, 2.0);
    clock_msg_->P[1] = 0.0;
    clock_msg_->P[2] = 0.0;

    clock_msg_->P[3] = 0.0;
    clock_msg_->P[4] = pow(RTC_CLK_SYNC_X1_OFFSET / 3.0, 2.0);
    clock_msg_->P[5] = 0.0;

    clock_msg_->P[6] = 0.0;
    clock_msg_->P[7] = 0.0;
    clock_msg_->P[8] = pow(RTC_CLK_SYNC_X2_OFFSET / 3.0, 2.0);
  } else {
    // Propagate filter.
    // Prediction.
    clock_msg_->dt = computeDt();

    predictX(clock_msg_->dt, static_cast<float>(clock_msg_->dac),
             clock_msg_->x[0], clock_msg_->x[1], clock_msg_->x[2], x_pred_);
    predictP(clock_msg_->dt, clock_msg_->P[0], clock_msg_->P[1],
             clock_msg_->P[2], clock_msg_->P[3], clock_msg_->P[4],
             clock_msg_->P[5], clock_msg_->P[6], clock_msg_->P[7],
             clock_msg_->P[8], static_cast<float>(clock_msg_->dac), Q_[0],
             Q_[1], Q_[2], clock_msg_->x[1], clock_msg_->x[2], P_pred_);

    // Measurement update.
    z_[0] = toUSec(clock_msg_->receive_time - clock_msg_->remote_time);
    computeResidual(x_pred_[0], z_[0], residual_);
    computeSInverse(P_pred_[0], R_, S_inv_);
    computeK(P_pred_[0], P_pred_[3], P_pred_[6], S_inv_[0], K_);
    estimateX(K_[0], K_[1], K_[2], residual_[0], x_pred_[0], x_pred_[1],
              x_pred_[2], clock_msg_->x);
    estimateP(K_[0], K_[1], K_[2], P_pred_[0], P_pred_[1], P_pred_[2],
              P_pred_[3], P_pred_[4], P_pred_[5], P_pred_[6], P_pred_[7],
              P_pred_[8], clock_msg_->P);
    clock_msg_->ppm =
        (DACTOVOLT * clock_msg_->dac - clock_msg_->x[1]) * clock_msg_->x[2];
  }

  last_update_ = clock_msg_->receive_time;
}

void ExternalClock::controlClock() {
  // Hard reset RTC if more than one second off.
  if (abs(measured_offset_s_) > 1.0) {
    RtcSync::getInstance().setSec(clock_msg_->remote_time.sec);

    char info[255];
    sprintf(info, "Set RTC. Sec: %u, NSec: %u", clock_msg_->remote_time.sec,
            clock_msg_->remote_time.nsec);
    nh_->loginfo(info);

    resetFilter();
  } else if (clock_msg_->dt > 0.0) {
#ifdef RTC_CLK_SYNC
    // LQR control.
    float dac = -(RTC_CLK_SYNC_LQR_GAIN * clock_msg_->x[0]);
    dac += clock_msg_->x[1] * VOLTTODAC; // Trim.
    dac = dac < RTC_CLK_SYNC_DAC_MIN ? RTC_CLK_SYNC_DAC_MIN : dac;
    dac = dac > RTC_CLK_SYNC_DAC_MAX ? RTC_CLK_SYNC_DAC_MAX : dac;
    clock_msg_->dac = static_cast<uint16_t>(roundf(dac));

    // Apply clock stabilization.
    while (DAC->STATUS.bit.SYNCBUSY) {
    }
    DAC->DATA.reg = clock_msg_->dac;
    while (DAC->STATUS.bit.SYNCBUSY) {
    }

    // Control RTC offset.
    if (fabsf(clock_msg_->x[0]) < RTC_CLK_SYNC_CONV_CRIT) {
      if (counter_converged_ != RTC_CLK_SYNC_CONV_WINDOW)
        counter_converged_++;
    } else {
      counter_converged_ = 0;
    }
    clock_msg_->sync = (counter_converged_ >= RTC_CLK_SYNC_CONV_WINDOW);
#endif
  }
}

void ExternalClock::resetFilter() {
  if (clock_msg_) {
    *clock_msg_ = versavis::ExtClk();
    clock_msg_->dac = static_cast<uint16_t>(VOLTTODAC * RTC_CLK_SYNC_X1);
    clock_msg_->sync = false;
  }
  last_update_ = ros::Time();
}
