////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Tc4Synced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef Tc4Synced_h
#define Tc4Synced_h

#include "clock_sync/TcSynced.h"

class Tc4Synced : public TcSynced {
public:
  // Singleton implementation.
  inline static Tc4Synced &getInstance() {
    static Tc4Synced instance;
    return instance;
  }
  Tc4Synced(Tc4Synced const &) = delete;
  void operator=(Tc4Synced const &) = delete;

private:
  Tc4Synced();
  void setupExternalEventEvsys() const override;
};

#endif
