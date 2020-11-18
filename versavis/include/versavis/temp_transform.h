#ifndef VERSAVIS_TEMP_TRANSFORM_H_
#define VERSAVIS_TEMP_TRANSFORM_H_

#include "versavis/TemperatureMicro.h"
#include "versavis/topic_transform.h"

#include <sensor_msgs/Temperature.h>

namespace versavis {
class TempTransform : public TopicTransform<versavis::TemperatureMicro,
                                            sensor_msgs::Temperature> {
public:
  inline TempTransform(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private)
      : TopicTransform(nh, nh_private) {
    nh_private_.getParam("frame_id", out_.header.frame_id);
    nh_private_.getParam("var", out_.variance);
    nh_private_.getParam("scale", scale_);
    nh_private_.getParam("offset", offset_);
  }

private:
  inline void update(const versavis::TemperatureMicro::ConstPtr &in) override {
    out_.header.seq = in->number;
    out_.header.stamp = in->time.data;

    out_.temperature = scale_ * in->temperature + offset_;
  }

  double scale_ = 0.0;
  double offset_ = 0.0;
};

} // namespace versavis

#endif // VERSAVIS_TEMP_TRANSFORM_H_
