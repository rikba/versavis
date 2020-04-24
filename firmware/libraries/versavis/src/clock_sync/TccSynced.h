////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TccSynced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef TccSynced_h
#define TccSynced_h

#include <Arduino.h>

#include "clock_sync/MeasurementStateExposure.h"
#include "clock_sync/TimerSynced.h"

class TccSynced : public TimerSynced {
public:
  // TODO(rikba): Add default values in C++14.
  struct ExposurePin {
    uint8_t group;
    uint8_t pin;
  };

  TccSynced(const MfrqPin &mfrq_pin, const ExposurePin &exp_pin, Tcc *tcc);

  void setupDataReady(const uint8_t port_group, const uint8_t pin,
                      const InterruptLogic &logic) override {}
  void setupMfrqWaveform() override;
  void setupExposure(const bool invert);

  void handleInterrupt() override;

  // Returns true only once per image.
  bool getTimeLastExposure(ros::Time *time, uint32_t *num, ros::Duration *exp);

  // Returns true only once per pps.
  bool getTimeLastPps(ros::Time *time, uint32_t *pps_num);

protected:
  virtual void setupExposureEvsys() const = 0;

  uint8_t getEventGeneratorId(const uint8_t pin) const;

  // Pointer to the actual timer.
  Tcc *tcc_ = NULL;

private:
  bool getExposurePinValue() const;

  void setup() const;
  void updateTopCompare() override;

  // Exposure state.
  MeasurementStateExposure exposure_state_;
  const ExposurePin exposure_pin_;

  // PPS state.
  MeasurementStateStamped pps_state_;

  // Wrap around state after half a clock cycle to handle ambiguities.
  ros::Time time_2_ = time_;
};

#endif
