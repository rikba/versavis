#include "clock_sync/Tcc2Synced.h"

// Setup MFRQ pin.
// TODO(rikba)
#define TCC2_MFRQ_GROUP 255
#define TCC2_MFRQ_PIN 255
#define TCC2_MFRQ_DRVSTR 0

// UNIQUE exposure definitions.
#define TCC2_EXP_CHANNEL 3
#define TCC2_EXP_GROUP PORTA
#define TCC2_EXP_PIN 21

Tcc2Synced::Tcc2Synced()
    : TccSynced(MfrqPin{.group = TCC2_MFRQ_GROUP,
                        .pin = TCC2_MFRQ_PIN,
                        .drvstr = TCC2_MFRQ_DRVSTR},
                ExposurePin{.group = TCC2_EXP_GROUP, .pin = TCC2_EXP_PIN},
                (Tcc *)TCC2) {
  // Enable interrupts.
  NVIC_SetPriority(TCC2_IRQn, 1);
  NVIC_EnableIRQ(TCC2_IRQn);
}

void Tcc2Synced::setupExposureEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TCC2_EXP_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC2_MC_1);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(TCC2_EXP_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(TCC2_EXP_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TCC2_EXP_CHANNEL)) {
  }
}

void TCC2_Handler() { Tcc2Synced::getInstance().handleInterrupt(); }
