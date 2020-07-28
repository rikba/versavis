#include "clock_sync/Tc5Synced.h"

// Setup MFRQ pin.
// TODO(rikba)
#define TC5_MFRQ_GROUP PORTA
#define TC5_MFRQ_PIN 4
#define TC5_MFRQ_DRVSTR 0

#define TC5_EXT_EVENT_CHANNEL 6

Tc5Synced::Tc5Synced()
    : TcSynced(MfrqPin{.group = TC5_MFRQ_GROUP,
                       .pin = TC5_MFRQ_PIN,
                       .drvstr = TC5_MFRQ_DRVSTR},
               (TcCount16 *)TC5) {
  // Enable interrupts.
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);
}

void Tc5Synced::setupExternalEventEvsys() const {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(TC5_EXT_EVENT_CHANNEL + 1) |
                    EVSYS_USER_USER(EVSYS_ID_USER_TC5_EVU);
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(getEventGeneratorId(TC5_MFRQ_PIN)) |
                       EVSYS_CHANNEL_CHANNEL(TC5_EXT_EVENT_CHANNEL);
  while (EVSYS->CHSTATUS.vec.CHBUSY & (1 << TC5_EXT_EVENT_CHANNEL)) {
  }
}

void TC5_Handler() { Tc5Synced::getInstance().handleInterrupt(); }
