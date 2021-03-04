////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ExternalClockGnss.h
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_ExternalClockGnss_h
#define Sensors_ExternalClockGnss_h

#include "nmea_parser/NmeaParser.h"
#include "sensors/ExternalClock.h"

class ExternalClockGnss : public ExternalClock {
public:
  ExternalClockGnss(ros::NodeHandle *nh, Uart *uart, const uint32_t baud_rate);

protected:
  RemoteTimeStatus setRemoteTime() override;

  Uart *uart_ = NULL;
  NmeaParser nmea_parser_;

  // State
  RemoteTimeStatus result_ = RemoteTimeStatus::kInitialize;
  bool received_time_ = false;
};

#endif
