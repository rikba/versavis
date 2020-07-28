#include "clock_sync/Tc3Synced.h"

// Setup MFRQ pin.
#define TC3_MFRQ_GROUP PORTA
#define TC3_MFRQ_PIN 14
#define TC3_MFRQ_DRVSTR 0

#define TC3_EXT_EVENT_CHANNEL 4

Tc3Synced::Tc3Synced()
    : TcSynced(MfrqPin{.group = TC3_MFRQ_GROUP,
                       .pin = TC3_MFRQ_PIN,
                       .drvstr = TC3_MFRQ_DRVSTR},
               (TcCount16 *)TC3) {
  // Enable interrupts.
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);
}

void Tc3Synced::setupExternalEventEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TC3_EXT_EVENT_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(TC3_MFRQ_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(TC3_EXT_EVENT_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TC3_EXT_EVENT_CHANNEL)) {
  }
}

void TC3_Handler() { Tc3Synced::getInstance().handleInterrupt(); }
