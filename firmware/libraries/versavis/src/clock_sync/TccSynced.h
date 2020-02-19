////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  TccSynced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef TccSynced_h
#define TccSynced_h

#include <Arduino.h>

#include "clock_sync/TimerSynced.h"

class TccSynced : public TimerSynced {
public:
  // TODO(rikba): Add default values in C++14.
  struct ExposurePin {
    uint8_t group;
    uint8_t pin;
  };
  struct ExposureState {
    uint32_t start = 0xFFFFFFFF;
    uint32_t stop = 0xFFFFFFFF;
    bool invert = false;

    inline bool isExposing() const { return start > stop; }
  };

  TccSynced(const MfrqPin &mfrq_pin, const ExposurePin &exp_pin, Tcc *tcc);

  void setupDataReady(const uint8_t port_group, const uint8_t pin,
                      const InterruptLogic &logic) override {}
  void setupMfrqWaveform() const override;
  void setupExposure(const bool invert);

  void handleInterrupt() override;

protected:
  virtual void setupExposureEvsys() const = 0;

  uint8_t getExposureEventGeneratorId() const;

private:
  bool getExposurePinValue() const;

  // Pointer to the actual timer.
  Tcc *tcc_ = NULL;

  void setup() const;

  // Exposure state.
  const ExposurePin exposure_pin_;
  ExposureState exposure_state_;
};

#endif
