////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  CamSynced.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for cameras in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_CamSynced_h
#define Sensors_CamSynced_h

#include "sensors/SensorSynced.h"
#include <RingBufCPP.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt32.h>

class CamSynced : public SensorSynced {
public:
  CamSynced(ros::NodeHandle *nh, TimerSynced *timer, const uint16_t rate_hz,
            const MeasurementState &meas_state);
  CamSynced(ros::NodeHandle *nh, TimerSynced *timer, const uint16_t rate_hz,
            const bool invert);
  void setupRos(const char *frame_id, const char *rate_topic,
                const char *seq_topic, const char *img_topic);

protected:
  std_msgs::Header *img_msg_ = NULL;
  RingBufCPP<std_msgs::Header, BUFFER_SIZE> buffer_;

private:
  void setSeqCb(const std_msgs::UInt32 &seq_msg);
};

#endif
