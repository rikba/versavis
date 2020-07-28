#include "clock_sync/Tc4Synced.h"

// Setup MFRQ pin.
// TODO(rikba)
#define TC4_MFRQ_GROUP PORTB
#define TC4_MFRQ_PIN 9
#define TC4_MFRQ_DRVSTR 0

#define TC4_EXT_EVENT_CHANNEL 5

Tc4Synced::Tc4Synced()
    : TcSynced(MfrqPin{.group = TC4_MFRQ_GROUP,
                       .pin = TC4_MFRQ_PIN,
                       .drvstr = TC4_MFRQ_DRVSTR},
               (TcCount16 *)TC4) {
  // Enable interrupts.
  NVIC_SetPriority(TC4_IRQn, 0);
  NVIC_EnableIRQ(TC4_IRQn);
}

void Tc4Synced::setupExternalEventEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TC4_EXT_EVENT_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(TC4_MFRQ_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(TC4_EXT_EVENT_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TC4_EXT_EVENT_CHANNEL)) {
  }
}

void TC4_Handler() { Tc4Synced::getInstance().handleInterrupt(); }
