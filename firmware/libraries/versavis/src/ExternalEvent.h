////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  ExternalEvent.h
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
  ExternalEvent(ros::NodeHandle *nh, const String &topic);
  void setup();
  void begin();
  void triggerMeasurement();
  void publish();
  void setupPublisher();

protected:
  // ------------ROS members-----------
  ros::NodeHandle *nh_;
  String topic_;
  ros::Publisher publisher_;

private:
  // Disable copy / assignment constructors.
  ExternalEvent(const ExternalEvent &) = delete;
  ExternalEvent &operator=(const ExternalEvent &) = delete;

  std_msgs::Time time_msg_;
};

#endif
