////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  GnssSync.h
////////////////////////////////////////////////////////////////////////////////
//
//  This sensor timestamps external events on the one of the AUX pins.
//  Tested pins: 2 (PA14)
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ExternalEvent_h
#define ExternalEvent_h

#include <std_msgs/Time.h>

#include "Sensor.h"

class ExternalEvent : public Sensor {
public:
  ExternalEvent(ros::NodeHandle *nh, const String &topic, const int rate_hz,
                Timer &timer);
  void setup() override;
  void begin() override;
  void triggerMeasurement() override;
  void publish() override;
  void setupPublisher() override;

private:
  // Disable copy / assignment constructors.
  ExternalEvent(const ExternalEvent &) = delete;
  ExternalEvent &operator=(const ExternalEvent &) = delete;

  const uint8_t kId = 0;

  std_msgs::Time range_msg_;
};

#endif
