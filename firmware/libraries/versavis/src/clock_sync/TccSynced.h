////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TccSynced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef TccSynced_h
#define TccSynced_h

#include <Arduino.h>

#include "clock_sync/MeasurementState.h"
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
  void setupMpwmWaveform() override;
  void updateRate(const uint16_t rate_hz) override;
  void setupExposure();

  void handleInterrupt() override;

  // Receive oldest PPS measurement. Only returns valid measurements. Clears
  // this and older measurements from buffer.
  bool getPpsMeasurement(Measurement *meas);
  inline void setPpsMeasurementState(const MeasurementState &state) {
    pps_state_ = state;
  }

protected:
  virtual void setupExposureEvsys() const = 0;

  uint8_t getEventGeneratorId(const uint8_t pin) const;

  // Pointer to the actual timer.
  Tcc *tcc_ = NULL;

  // PPS state.
  MeasurementState pps_state_;

private:
  bool getExposurePinValue() const;

  void setup() const;
  void updateTopCompare() override;

  // Exposure state.
  MeasurementState exposure_state_;
  const ExposurePin exposure_pin_;

  // Wrap around state after half a clock cycle to handle ambiguities.
  ros::Time time_2_ = time_;
  uint32_t time_2_cc_ = 0xFFFFFFFF; // the capture value at time_2_.
};

#endif
