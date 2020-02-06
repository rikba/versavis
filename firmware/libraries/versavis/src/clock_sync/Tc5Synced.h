////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Tc5Synced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef Tc5Synced_h
#define Tc5Synced_h

#include "clock_sync/TcSynced.h"

class Tc5Synced : public TcSynced {
public:
  // Singleton implementation.
  inline static Tc5Synced &getInstance() {
    static Tc5Synced instance;
    return instance;
  }
  Tc5Synced(Tc5Synced const &) = delete;
  void operator=(Tc5Synced const &) = delete;

private:
  Tc5Synced();

  void setupOutPin() override;
};

#endif
