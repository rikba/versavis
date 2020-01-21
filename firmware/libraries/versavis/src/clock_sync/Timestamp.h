////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Timestamp.h
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation to synchronize the VersaVIS against a GNSS receiver time.
//  This implementation assumes that the GNSS receiver is connected via PPS
//  signal and NMEA absolute time.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GnssSync_h
#define GnssSync_h

#include <ros.h>

class Timestamp {
public:
  Timestamp(ros::NodeHandle *nh) : nh_(nh) {}

  inline bool hasTime() { return has_time_; }
  bool getTime(ros::Time *time);
  void setTimeNow(const ros::Time &time);

private:
  ros::Time time_;
  bool has_time_ = false;
  ros::NodeHandle *nh_ = NULL;
};

#endif
