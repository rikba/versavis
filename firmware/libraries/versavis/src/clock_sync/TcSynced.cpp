#include "clock_sync/TcSynced.h"

#include "clock_sync/RtcSync.h"

#include "helper.h"
#include "versavis_configuration.h"

TcSynced::TcSynced(TcCount16 *tc) : tc_(tc) { setup(); }

void TcSynced::setup() const {
  DEBUG_PRINTLN("[TcSynced]: setupTimer.");
  if (!tc_) {
    error("NO_TIMER (TcSynced.cpp): timer does not exist.", 201);
  }

  DEBUG_PRINTLN("[TcSynced]: Disabling timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Setup EVCTRL to retrigger on RTC overflow.");
  tc_->EVCTRL.reg |= TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_RETRIGGER;

  DEBUG_PRINTLN("[TcSynced]: Capture channel 1.");
  tc_->CTRLC.reg |= TC_CTRLC_CPTEN1;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Enabling event interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC1 | TC_INTENSET_OVF;
  DEBUG_PRINTLN("[TcSynced]: Clearing interrupt flags.");
  tc_->INTFLAG.reg |= TC_INTENSET_MC1 | TC_INTFLAG_OVF;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
}

void TcSynced::setupMfrq(const uint16_t rate_hz, const bool invert) {
  // Set parameters.
  rate_hz_ = rate_hz;
  invert_trigger_ = invert;
  prescaler_ = RtcSync::getInstance().findMinPrescalerFrq(rate_hz, top_);
  RtcSync::getInstance().computeFrq(rate_hz, kPrescalers[prescaler_], &top_);

  // Setup wavegen.
  DEBUG_PRINTLN("[TcSynced]: Disabling timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINT("[TcSynced]: Prescaling timer by ");
  DEBUG_PRINTLN(kPrescalers[prescaler_]);
  tc_->CTRLA.reg |= TC_CTRLA_PRESCALER(prescaler_);
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Activate MFRQ.");
  tc_->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Make channel 0 compare register.");
  tc_->CTRLC.reg &= ~TC_CTRLC_CPTEN0;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  if (invert) {
    tc_->CTRLC.reg |= TC_CTRLC_INVEN0;
    while (tc_->STATUS.bit.SYNCBUSY) {
    }
  }

  DEBUG_PRINT("[TcSynced]: Set FRQ top: ");
  DEBUG_PRINTLN(top_);
  tc_->CC[0].reg = top_;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  DEBUG_PRINTLN("[TcSynced]: Enabling MFRQ interrupts.");
  tc_->INTENSET.reg |= TC_INTENSET_MC0;
  tc_->INTFLAG.reg |= TC_INTENSET_MC0;

  DEBUG_PRINTLN("[TcSynced]: Enable timer.");
  while (tc_->STATUS.bit.SYNCBUSY) {
  }
  tc_->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc_->STATUS.bit.SYNCBUSY) {
  }

  // Setup output pin.
  REG_PM_APBBMASK |= PM_APBBMASK_PORT; // Port ABP Clock Enable.
  setupOutPin();
}

void TcSynced::setupDataReady(const uint8_t port_group, const uint8_t pin,
                              const InterruptLogic &logic) {
  // Store parameters.
  dr_port_group_ = port_group;
  dr_pin_ = pin;

  // PORT configurations.
  REG_PM_APBBMASK |= PM_APBBMASK_PORT;

  DEBUG_PRINTLN("[TcSynced]: Configure pin input.");
  PORT->Group[port_group].DIRCLR.reg = PORT_DIRCLR_DIRCLR(1 << pin);

  DEBUG_PRINTLN("[TcSynced]: Connect data ready pin with PMUX A.");
  if (pin % 2) {
    PORT->Group[port_group].PMUX[pin >> 1].reg |= PORT_PMUX_PMUXO_A; // Odd
  } else {
    PORT->Group[port_group].PMUX[pin >> 1].reg |= PORT_PMUX_PMUXE_A; // Even
  }
  PORT->Group[port_group].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[port_group].PINCFG[pin].reg |= PORT_PINCFG_INEN;

  // Activate EIC edge detection.
  if (logic == InterruptLogic::kRise || logic == InterruptLogic::kFall ||
      logic == InterruptLogic::kBoth) {
    DEBUG_PRINTLN("[TcSynced]: Enabling generic clock 4 for edge detection.");
    GCLK->CLKCTRL.reg =
        GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_EIC;
    while (GCLK->STATUS.bit.SYNCBUSY) {
    } // Wait for synchronization}
  }

  // EIC configurations.
  REG_PM_APBAMASK |= PM_APBAMASK_EIC;
  DEBUG_PRINTLN("[TcSynced]: Disable EIC.");
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
  EIC->CTRL.reg &= ~EIC_CTRL_ENABLE;
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  DEBUG_PRINTLN("[TcSynced]: Configure EXTINTEO.");
  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO(1 << (pin % 16));

  uint8_t config_id = pin < 16 ? 1 : 2;
  config_id = pin < 8 ? 0 : config_id;

  DEBUG_PRINTLN("[TcSynced]: Configure SENSE.");
  DEBUG_PRINTLN(config_id);
  uint8_t id = pin % 8;
  uint32_t logic_val = static_cast<uint32_t>(logic);
  switch (id) {
  case 0:
    EIC->CONFIG[config_id].bit.SENSE0 = logic_val;
    break;
  case 1:
    EIC->CONFIG[config_id].bit.SENSE1 = logic_val;
    break;
  case 2:
    EIC->CONFIG[config_id].bit.SENSE2 = logic_val;
    break;
  case 3:
    EIC->CONFIG[config_id].bit.SENSE3 = logic_val;
    break;
  case 4:
    EIC->CONFIG[config_id].bit.SENSE4 = logic_val;
    break;
  case 5:
    EIC->CONFIG[config_id].bit.SENSE5 = logic_val;
    break;
  case 6:
    EIC->CONFIG[config_id].bit.SENSE6 = logic_val;
    break;
  case 7:
    EIC->CONFIG[config_id].bit.SENSE7 = logic_val;
    break;
  }

  DEBUG_PRINTLN("[TcSynced]: Enable interrupt handler.");
  EIC->INTENSET.reg |= EIC_INTENSET_EXTINT(1 << (pin % 16));
  EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(1 << (pin % 16));

  DEBUG_PRINTLN("[TcSynced]: Enable EIC.");
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization
  EIC->CTRL.reg |= EIC_CTRL_ENABLE;
  while (EIC->STATUS.bit.SYNCBUSY) {
  } // Wait for synchronization

  NVIC_SetPriority(EIC_IRQn, 0x02);
  NVIC_EnableIRQ(EIC_IRQn);
}

void TcSynced::handleInterrupt() {
  if (tc_->INTFLAG.bit.MC0 && (getOutPinValue() ^ invert_trigger_)) {
    trigger();
  }
  if (tc_->INTFLAG.bit.MC1) {
    syncRtc();
  } else if (tc_->INTFLAG.bit.OVF) {
    overflow();
  }

  // Clear flags.
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.MC0;
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.MC1;
  tc_->INTFLAG.reg |= tc_->INTFLAG.bit.OVF;
}
