////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Tcc1Synced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef Tcc1Synced_h
#define Tcc1Synced_h

#include "clock_sync/TccSynced.h"

class Tcc1Synced : public TccSynced {
public:
  // Singleton implementation.
  inline static Tcc1Synced &getInstance() {
    static Tcc1Synced instance;
    return instance;
  }
  Tcc1Synced(Tcc1Synced const &) = delete;
  void operator=(Tcc1Synced const &) = delete;

private:
  Tcc1Synced();

  void setupExposureEvsys() const override;
};

#endif
