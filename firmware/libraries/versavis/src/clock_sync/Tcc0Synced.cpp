#include "clock_sync/Tcc0Synced.h"

// Setup MFRQ pin.
#define TCC0_MFRQ_GROUP PORTA
#define TCC0_MFRQ_PIN 4
#define TCC0_MFRQ_DRVSTR 1

// UNIQUE exposure definitions.
#define TCC0_EXP_CHANNEL 1
#define TCC0_EXP_GROUP PORTA
#define TCC0_EXP_PIN 15

Tcc0Synced::Tcc0Synced()
    : TccSynced(MfrqPin{.group = TCC0_MFRQ_GROUP,
                        .pin = TCC0_MFRQ_PIN,
                        .drvstr = TCC0_MFRQ_DRVSTR},
                ExposurePin{.group = TCC0_EXP_GROUP, .pin = TCC0_EXP_PIN},
                (Tcc *)TCC0) {
  // This is a 32 bit counter.
  top_ = 0xFFFFFFFF;
  // Enable interrupts. Not as high priority as the RTC interrupt.
  NVIC_SetPriority(TCC0_IRQn, 0x01);
  NVIC_EnableIRQ(TCC0_IRQn);
}

void Tcc0Synced::setupExposureEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TCC0_EXP_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_1);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getExposureEventGeneratorId()) |
                       EVSYS_CHANNEL_CHANNEL(TCC0_EXP_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TCC0_EXP_CHANNEL)) {
  }
}

void TCC0_Handler() { Tcc0Synced::getInstance().handleInterrupt(); }
