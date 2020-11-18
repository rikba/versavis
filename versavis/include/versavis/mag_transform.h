#ifndef VERSAVIS_MAG_TRANSFORM_H_
#define VERSAVIS_MAG_TRANSFORM_H_

#include "versavis/MagneticMicro.h"
#include "versavis/topic_transform.h"

#include <sensor_msgs/MagneticField.h>

namespace versavis {
class MagTransform : public TopicTransform<versavis::MagneticMicro,
                                           sensor_msgs::MagneticField> {
public:
  inline MagTransform(const ros::NodeHandle &nh,
                      const ros::NodeHandle &nh_private)
      : TopicTransform(nh, nh_private) {
    nh_private_.getParam("frame_id", out_.header.frame_id);

    out_.magnetic_field_covariance[0] = -1.0;
    nh_private_.getParam("cov", out_.magnetic_field_covariance[0]);
    nh_private_.getParam("cov", out_.magnetic_field_covariance[4]);
    nh_private_.getParam("cov", out_.magnetic_field_covariance[8]);

    nh_private_.getParam("scale", scale_);
  }

private:
  inline void update(const versavis::MagneticMicro::ConstPtr &in) override {
    out_.header.seq = in->number;
    out_.header.stamp = in->time.data;

    out_.magnetic_field.x = scale_ * in->mx;
    out_.magnetic_field.y = scale_ * in->my;
    out_.magnetic_field.z = scale_ * in->mz;
  }

  double scale_ = 0.0;
};

} // namespace versavis

#endif // VERSAVIS_MAG_TRANSFORM_H_
