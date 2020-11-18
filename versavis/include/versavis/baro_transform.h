#ifndef VERSAVIS_BARO_TRANSFORM_H_
#define VERSAVIS_BARO_TRANSFORM_H_

#include "versavis/PressureMicro.h"
#include "versavis/topic_transform.h"

#include <sensor_msgs/FluidPressure.h>

namespace versavis {
class BaroTransform : public TopicTransform<versavis::PressureMicro,
                                            sensor_msgs::FluidPressure> {
public:
  inline BaroTransform(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private)
      : TopicTransform(nh, nh_private) {
    nh_private_.getParam("frame_id", out_.header.frame_id);
    nh_private_.getParam("var", out_.variance);
    nh_private_.getParam("scale", scale_);
  }

private:
  inline void update(const versavis::PressureMicro::ConstPtr &in) override {
    out_.header.seq = in->number;
    out_.header.stamp = in->time.data;

    out_.fluid_pressure = scale_ * in->pressure;
  }

  double scale_ = 0.0;
};

} // namespace versavis

#endif // VERSAVIS_BARO_TRANSFORM_H_
