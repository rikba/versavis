////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Adis16448BmlzTriggered.h
////////////////////////////////////////////////////////////////////////////////
//
//  Triggered Adis16448 implementation.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_Adis16448BmlzTriggered_h
#define Sensors_Adis16448BmlzTriggered_h

#include "drivers/Adis16448.h"
#include "sensors/ImuSynced.h"

class Adis16448BmlzTriggered : public ImuSynced {
public:
  Adis16448BmlzTriggered(TimerSynced *timer, const uint16_t rate_hz,
                         const uint8_t dr_port_group, const uint8_t dr_pin,
                         const uint8_t chip_select);
  void publish() override;

private:
  Adis16448 imu_;
};

#endif
