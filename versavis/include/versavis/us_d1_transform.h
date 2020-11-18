#ifndef VERSAVIS_US_D1_TRANSFORM_H_
#define VERSAVIS_US_D1_TRANSFORM_H_

#include "versavis/UsD1Micro.h"
#include "versavis/topic_transform.h"

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <versavis/UsD1.h>

namespace versavis {
class UsD1Transform
    : public TopicTransform<versavis::UsD1Micro, versavis::UsD1> {
public:
  inline UsD1Transform(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private)
      : TopicTransform(nh, nh_private) {
    nh_private_.getParam("frame_id", out_.range.header.frame_id);
    // TODO(rikba): Fix sensor type https://github.com/ros/common_msgs/pull/153
    out_.range.radiation_type = sensor_msgs::Range::INFRARED;

    // See
    // https://cdn.shopify.com/s/files/1/0113/0414/0900/files/User_Manual_US-D1.pdf?16288212927919010227
    nh_private_.getParam("fov", out_.range.field_of_view);
    nh_private_.getParam("min_range", out_.range.min_range);
    nh_private_.getParam("max_range", out_.range.max_range);
    nh_private_.getParam("var_low", var_low_);
    nh_private_.getParam("var_high", var_high_);

    nh_private_.getParam("scale", scale_);
    nh_private_.getParam("close_range", close_range_);
  }

private:
  inline void update(const versavis::UsD1Micro::ConstPtr &in) override {
    out_.range.header.seq = in->number;
    out_.range.header.stamp = in->time.data;
    out_.range.range = scale_ * in->range;
    out_.snr = in->snr;
    out_.variance = out_.range.range < close_range_ ? var_low_ : var_high_;
  }

  double scale_ = 0.0;
  double close_range_ = 0.0;
  double var_low_ = -1.0;
  double var_high_ = -1.0;
};

} // namespace versavis

#endif // VERSAVIS_US_D1_TRANSFORM_H_
