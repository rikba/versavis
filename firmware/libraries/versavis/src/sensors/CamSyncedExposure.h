////////////////////////////////////////////////////////////////////////////////
//  February 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Adis16448BmlzTriggered.h
////////////////////////////////////////////////////////////////////////////////
//
//  Triggered camera with exposure implementation.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Sensors_CamSyncedExposure_h
#define Sensors_CamSyncedExposure_h

#include "clock_sync/TccSynced.h"
#include "sensors/CamSynced.h"

class CamSyncedExposure : public CamSynced {
public:
  CamSyncedExposure(ros::NodeHandle *nh, TccSynced *timer,
                    const uint16_t rate_hz, const bool invert_trigger,
                    const bool invert_exposure);
  bool publish() override;

protected:
  void setSeqCb(const std_msgs::UInt32 &seq_msg) override;

private:
  void compensateExposure();
  ros::Time expected_stamp_;
  ros::Time prev_expected_stamp_;
};

#endif
