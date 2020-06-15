#include "clock_sync/Tcc0Synced.h"

#include "helper.h"

// Setup MFRQ pin.
#define TCC0_MFRQ_GROUP PORTA
#define TCC0_MFRQ_PIN 4
#define TCC0_MFRQ_DRVSTR 1

// UNIQUE exposure definitions.
#define TCC0_EXP_CHANNEL 1
#define TCC0_EXP_GROUP PORTA
#define TCC0_EXP_PIN 15

// Setup PPS pin.
#define PPS_CHANNEL 4 // UNIQUE event channel.
#define PPS_GROUP PORTA
#define PPS_PIN 11

Tcc0Synced::Tcc0Synced()
    : TccSynced(MfrqPin{.group = TCC0_MFRQ_GROUP,
                        .pin = TCC0_MFRQ_PIN,
                        .drvstr = TCC0_MFRQ_DRVSTR},
                ExposurePin{.group = TCC0_EXP_GROUP, .pin = TCC0_EXP_PIN},
                (Tcc *)TCC0),
      pps_pin_{.group = PPS_GROUP, .pin = PPS_PIN} {
  // This is a 24 bit counter.
  top_max_ = 0xFFFFFF;
  top_ = top_max_;
  setupAmbiguityComparison();
  // Enable interrupts.
  NVIC_SetPriority(TCC0_IRQn, 0);
  NVIC_EnableIRQ(TCC0_IRQn);
}

void Tcc0Synced::setupExposureEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TCC0_EXP_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_1);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(TCC0_EXP_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(TCC0_EXP_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TCC0_EXP_CHANNEL)) {
  }
}

void Tcc0Synced::setupPps(const bool invert) {
  DEBUG_PRINT("[Tcc0Synced]: Configuring pps pin ");
  DEBUG_PRINT(pps_pin_.pin);
  DEBUG_PRINT(" of group ");
  DEBUG_PRINTLN(pps_pin_.group);

  DEBUG_PRINTLN("[Tcc0Synced]: Setup pps interrupt pin.");
  const auto logic = invert ? InterruptLogic::kFall : InterruptLogic::kRise;
  setupInterruptPin(pps_pin_.group, pps_pin_.pin, logic, true);
  setupPpsEvsys();

  // Setup interrupt
  DEBUG_PRINTLN("[Tcc0Synced]: Disabling timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  DEBUG_PRINTLN("[Tcc0Synced]: Configure pps capture and interrupt.");
  tcc_->EVCTRL.reg |= TCC_EVCTRL_MCEI2;
  DEBUG_PRINTLN("[Tcc0Synced]: TCC_CTRLA_CPTEN2.");
  tcc_->CTRLA.reg |= TCC_CTRLA_CPTEN2;
  DEBUG_PRINTLN("[Tcc0Synced]: TCC_INTENSET_MC2.");
  tcc_->INTENSET.reg |= TCC_INTENSET_MC2;
  DEBUG_PRINTLN("[Tcc0Synced]: TCC_INTFLAG_MC2.");
  tcc_->INTFLAG.reg |= TCC_INTFLAG_MC2;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[Tcc0Synced]: Enable timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
}

void Tcc0Synced::setupPpsEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(PPS_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_2);

  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(PPS_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(PPS_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << PPS_CHANNEL)) {
  }
}

void Tcc0Synced::setupAmbiguityComparison() const {
  DEBUG_PRINTLN("[Tcc0Synced]: Disabling timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[Tcc0Synced]: Make CC3 compare register.");
  tcc_->CTRLA.reg &= ~TCC_CTRLA_CPTEN3;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }

  DEBUG_PRINTLN("[Tcc0Synced]: Enabling half top interrupts.");
  tcc_->INTENSET.reg |= TCC_INTENSET_MC3;
  tcc_->INTFLAG.reg |= TCC_INTFLAG_MC3;

  DEBUG_PRINTLN("[Tcc0Synced]: Enable timer.");
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
  tcc_->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (tcc_->SYNCBUSY.bit.ENABLE) {
  }
}

void TCC0_Handler() { Tcc0Synced::getInstance().handleInterrupt(); }
