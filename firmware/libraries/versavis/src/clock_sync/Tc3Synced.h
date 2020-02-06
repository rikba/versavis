////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Tc3Synced.h
////////////////////////////////////////////////////////////////////////////////

#ifndef Tc3Synced_h
#define Tc3Synced_h

#include "clock_sync/TcSynced.h"

class Tc3Synced : public TcSynced {
public:
  // Singleton implementation.
  inline static Tc3Synced &getInstance() {
    static Tc3Synced instance;
    return instance;
  }
  Tc3Synced(Tc3Synced const &) = delete;
  void operator=(Tc3Synced const &) = delete;

private:
  Tc3Synced();

  void setupOutPin() override;
  virtual bool getOutPinValue() const override;
};

#endif
