#include "clock_sync/RtcSync.h"
#include "clock_sync/Tcc0Synced.h"
#include "sensors/ExternalClock.h"

ExternalClock::ExternalClock() : SensorSynced(&Tcc0Synced::getInstance()) {
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

  DAC->DATA.reg |= computeDacData(RTC_CTRL_V_NOM); // 1.5V nominal current
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

void ExternalClock::setupRos(ros::NodeHandle *nh, const char *topic) {
  // Create static ROS message.
  static versavis::ExtClk clock_msg;
  clock_msg_ = &clock_msg;

  if (nh) {

    // Create static ROS publisher.
    static ros::Publisher pub(topic, clock_msg_);
    publisher_ = &pub;

    // Advertise.
    nh->advertise(pub);
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

  // Sommer, Hannes, et al. "A low-cost system for high-rate, high-accuracy
  // temporal calibration for LIDARs and cameras." 2017 IEEE/RSJ International
  // Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.
  // https://github.com/ethz-asl/cuckoo_time_translator/blob/master/cuckoo_time_translator_algorithms/src/KalmanOwt.cpp#L51-L88

  if (isInitializing()) {
    // Initialize filter.
    auto offset =
        computeDuration(clock_msg_->remote_time, clock_msg_->receive_time);
    measured_offset_s_ = offset.toSec();
    clock_msg_->x[0] = toUSec(offset);
    clock_msg_->x[1] = 0.0;
    clock_msg_->P[0] = pow(RTC_INITIAL_OFFSET * 1.0e6, 2.0);
    clock_msg_->P[1] = 0.0;
    clock_msg_->P[2] = 0.0;
    clock_msg_->P[3] = pow(RTC_MAX_SKEW, 2.0);
  } else {
    // Propagate filter.
    // Prediction.
    clock_msg_->dt = computeDt();

    // x(k) = F(k) * x(k-1)
    clock_msg_->x[0] += clock_msg_->dt * clock_msg_->x[1];

    // P(k) = F(k) * P(k-1) * F(k).transpose + dt * Q
    clock_msg_->P[0] +=
        Q_[0] * clock_msg_->dt + clock_msg_->P[2] * clock_msg_->dt +
        clock_msg_->dt * (clock_msg_->P[1] + clock_msg_->P[3] * clock_msg_->dt);
    clock_msg_->P[1] += clock_msg_->P[3] * clock_msg_->dt;
    clock_msg_->P[2] += clock_msg_->P[3] * clock_msg_->dt;
    clock_msg_->P[3] += Q_[1] * clock_msg_->dt;

    // Measurement update.
    const float S_inv = 1.0 / (R_ + clock_msg_->P[0]);
    float K[2];
    K[0] = clock_msg_->P[0] * S_inv;
    K[1] = clock_msg_->P[2] * S_inv;

    const float residual = toUSec(computeDuration(clock_msg_->remote_time,
                                                  clock_msg_->receive_time)) -
                           clock_msg_->x[0];

    clock_msg_->x[0] += K[0] * residual;
    clock_msg_->x[1] += K[1] * residual;

    clock_msg_->P[2] -= K[1] * clock_msg_->P[0];
    clock_msg_->P[3] -= K[1] * clock_msg_->P[1];
    clock_msg_->P[0] -= K[0] * clock_msg_->P[0];
    clock_msg_->P[1] -= K[0] * clock_msg_->P[1];
  }

  last_update_ = clock_msg_->receive_time;
}

void ExternalClock::controlClock() {
  // Hard reset RTC if more than one second off.
  // TODO(rikba): Replace measurement with offset estimation.
  if (abs(measured_offset_s_) > 1.0) {
    ros::Time reset_time = RtcSync::getInstance().getTimeNow();
    ros::Duration offset;
    offset.fromSec(measured_offset_s_);
    reset_time -= offset;
    RtcSync::getInstance().setTime(reset_time);
    resetFilter();
  } else if (clock_msg_->dt > 0.0) {
    // Calculate error terms.
    clock_msg_->e = -clock_msg_->x[0] * RTC_CTRL_RANGE_INV / clock_msg_->dt;

    clock_msg_->i *= RTC_CTRL_I_DECAY;
    clock_msg_->i += clock_msg_->e * clock_msg_->dt;
    clock_msg_->i =
        clock_msg_->i > RTC_CTRL_I_MAX ? RTC_CTRL_I_MAX : clock_msg_->i;
    clock_msg_->i =
        clock_msg_->i < -RTC_CTRL_I_MAX ? -RTC_CTRL_I_MAX : clock_msg_->i;

    clock_msg_->d = -clock_msg_->x[1] * RTC_CTRL_RANGE_INV;

    // Calculate control input.
    clock_msg_->u = RTC_CTRL_KP * clock_msg_->e + RTC_CTRL_KD * clock_msg_->d +
                    RTC_CTRL_KI * clock_msg_->i;
    // Clamp control input.
    clock_msg_->u = clock_msg_->u > 1.0 ? 1.0 : clock_msg_->u;
    clock_msg_->u = clock_msg_->u < -1.0 ? -1.0 : clock_msg_->u;
    clock_msg_->dac = computeDacData(RTC_CTRL_V_NOM + clock_msg_->u);

    // Apply clock stabilization.
    while (DAC->STATUS.bit.SYNCBUSY) {
    }
    DAC->DATA.reg = clock_msg_->dac;
    while (DAC->STATUS.bit.SYNCBUSY) {
    }

    // Control RTC offset.
    if (fabsf(clock_msg_->x[0]) < RTC_CTRL_CONV_CRIT) {
      if (counter_converged_ != RTC_CTRL_CONV_WINDOW)
        counter_converged_++;
    } else {
      counter_converged_ = 0;
    }
    clock_msg_->sync = (counter_converged_ >= RTC_CTRL_CONV_WINDOW);
    // if (clock_msg_->sync) {
    //  RtcSync::getInstance().setOffset(
    //      static_cast<int32_t>(clock_msg_->x[0] * 1.0e3));
    //  clock_msg_->x[0] = 0;
    //}
  }
}

void ExternalClock::resetFilter() {
  if (clock_msg_) {
    *clock_msg_ = versavis::ExtClk();
    clock_msg_->dac = computeDacData(clock_msg_->u);
    clock_msg_->sync = false;
  }
  last_update_ = ros::Time();
}
