////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Tcc0Synced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef Tcc0Synced_h
#define Tcc0Synced_h

#include "clock_sync/TccSynced.h"

class Tcc0Synced : public TccSynced {
public:
  struct PpsPin {
    uint8_t group;
    uint8_t pin;
  };

  // Singleton implementation.
  inline static Tcc0Synced &getInstance() {
    static Tcc0Synced instance;
    return instance;
  }
  Tcc0Synced(Tcc0Synced const &) = delete;
  void operator=(Tcc0Synced const &) = delete;

  void setupPps();

private:
  Tcc0Synced();
  void setupPpsEvsys() const;

  void setupExposureEvsys() const override;

  void setupAmbiguityComparison() const;

  const PpsPin pps_pin_;
};

#endif
