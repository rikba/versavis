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
#include "clock_sync/Timestamp.h"

class TccSynced : public TimerSynced {
public:
  // TODO(rikba): Add default values in C++14.
  struct ExposurePin {
    uint8_t group;
    uint8_t pin;
  };
  class ExposureState {
  public:
    inline void syncRtc() {
      start_capture_.syncRtc();
      stop_capture_.syncRtc();
    }
    inline void overflow() {
      start_capture_.overflow();
      stop_capture_.overflow();
    }
    inline void startExposure(const uint32_t cc, const uint8_t prescaler,
                              const uint32_t top) {
      start_capture_.setTicks(cc);
      start_capture_.computeTime(prescaler, top, &start_time_);
      has_image_ = false; // Invalidate current image.
    }
    inline void stopExposure(const uint32_t cc, const uint8_t prescaler,
                             const uint32_t top) {
      stop_capture_.setTicks(cc);
      has_image_ = stop_capture_.computeTime(prescaler, top, &stop_time_);
      image_counter_++;
    }

    inline bool getTime(ros::Time *time, ros::Duration *exp,
                        uint32_t *img_num) {
      if (has_image_) {
        has_image_ = false;
        ros::Duration duration = computeDuration(start_time_, stop_time_);
        bool valid = ((duration.sec > 0) ||
                      ((duration.sec == 0) && (duration.nsec > 0)));

        if (exp && valid) {
          *exp = duration;
        }

        if (time && valid) {
          *time = start_time_;
          duration *= 0.5;
          *time += duration;
        }

        if (img_num && valid) {
          *img_num = image_counter_;
        }

        return valid;
      } else {
        return false;
      }
    }

    bool invert_ = false;

  private:
    Timestamp start_capture_;
    Timestamp stop_capture_;

    ros::Time start_time_;
    ros::Time stop_time_;

    bool has_image_ = false;

    uint32_t image_counter_ = 0xFFFFFFFF;
  };

  class PpsState {
  public:
    inline void syncRtc() { pps_.syncRtc(); }
    inline void overflow() { pps_.overflow(); }
    inline void receive(const uint32_t cc, const uint8_t prescaler,
                        const uint32_t top) {
      pps_counter_++;
      pps_.setTicks(cc);
      has_pps_ = pps_.computeTime(prescaler, top, &pps_time_);
    }

    inline bool getTime(ros::Time *time, uint32_t *pps_num) {
      if (has_pps_) {
        has_pps_ = false;
        if (time)
          *time = pps_time_;
        if (pps_num)
          *pps_num = pps_counter_;
        return true;
      } else {
        return false;
      }
    }

  private:
    Timestamp pps_;
    ros::Time pps_time_;
    bool has_pps_ = false;

    uint32_t pps_counter_ = 0xFFFFFFFF;
  };

  TccSynced(const MfrqPin &mfrq_pin, const ExposurePin &exp_pin, Tcc *tcc);

  void setupDataReady(const uint8_t port_group, const uint8_t pin,
                      const InterruptLogic &logic) override {}
  void setupMfrqWaveform() const override;
  void setupExposure(const bool invert);

  void handleInterrupt() override;

  // Returns true only once per image.
  inline bool computeTimeLastMidExposure(ros::Time *time, ros::Duration *exp,
                                         uint32_t *img_num) {
    return exposure_state_.getTime(time, exp, img_num);
  }

  // Returns true only once per pps.
  inline bool getTimeLastPps(ros::Time *time, uint32_t *pps_num) {
    return pps_state_.getTime(time, pps_num);
  }

protected:
  virtual void setupExposureEvsys() const = 0;

  uint8_t getEventGeneratorId(const uint8_t pin) const;

  // Pointer to the actual timer.
  Tcc *tcc_ = NULL;

private:
  bool getExposurePinValue() const;

  void setup() const;

  // Exposure state.
  ExposureState exposure_state_;
  const ExposurePin exposure_pin_;

  // PPS state.
  PpsState pps_state_;
};

#endif
