////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Tcc2Synced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef Tcc2Synced_h
#define Tcc2Synced_h

#include "clock_sync/TccSynced.h"

class Tcc2Synced : public TccSynced {
public:
  // Singleton implementation.
  inline static Tcc2Synced &getInstance() {
    static Tcc2Synced instance;
    return instance;
  }
  Tcc2Synced(Tcc2Synced const &) = delete;
  void operator=(Tcc2Synced const &) = delete;

private:
  Tcc2Synced();
};

#endif
