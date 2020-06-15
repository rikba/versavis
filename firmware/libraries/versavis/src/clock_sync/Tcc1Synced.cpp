#include "clock_sync/Tcc1Synced.h"

// Setup MFRQ pin.
#define TCC1_MFRQ_GROUP PORTA
#define TCC1_MFRQ_PIN 7
#define TCC1_MFRQ_DRVSTR 0

// UNIQUE exposure definitions.
#define TCC1_EXP_CHANNEL 2
#define TCC1_EXP_GROUP PORTA
#define TCC1_EXP_PIN 20

Tcc1Synced::Tcc1Synced()
    : TccSynced(MfrqPin{.group = TCC1_MFRQ_GROUP,
                        .pin = TCC1_MFRQ_PIN,
                        .drvstr = TCC1_MFRQ_DRVSTR},
                ExposurePin{.group = TCC1_EXP_GROUP, .pin = TCC1_EXP_PIN},
                (Tcc *)TCC1) {
  // This is a 24 bit counter.
  top_max_ = 0xFFFFFF;
  top_ = top_max_;
  // Enable interrupts.
  NVIC_SetPriority(TCC1_IRQn, 0);
  NVIC_EnableIRQ(TCC1_IRQn);
}

void Tcc1Synced::setupExposureEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TCC1_EXP_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_1);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(TCC1_EXP_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(TCC1_EXP_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TCC1_EXP_CHANNEL)) {
  }
}

void TCC1_Handler() { Tcc1Synced::getInstance().handleInterrupt(); }
