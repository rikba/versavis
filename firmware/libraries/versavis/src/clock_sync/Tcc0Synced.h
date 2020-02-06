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
  // Singleton implementation.
  inline static Tcc0Synced &getInstance() {
    static Tcc0Synced instance;
    return instance;
  }
  Tcc0Synced(Tcc0Synced const &) = delete;
  void operator=(Tcc0Synced const &) = delete;

private:
  Tcc0Synced();
  void setupOutPin() override;
};

#endif
