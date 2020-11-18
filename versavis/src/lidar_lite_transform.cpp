#include "versavis/LidarLiteMicro.h"
#include "versavis/topic_transform.h"

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <versavis/LidarLite.h>

namespace versavis {
class LidarLiteTransform
    : public TopicTransform<versavis::LidarLiteMicro, versavis::LidarLite> {
  inline LidarLiteTransform() {
    nh_private_.param("frame_id", out_.range.header.frame_id);
    out_.range.radiation_type = sensor_msgs::Range::INFRARED;

    nh_private_.param("fov", out_.range.field_of_view);
    nh_private_.param("min_range", out_.range.min_range);
    nh_private_.param("max_range", out_.range.max_range);
    nh_private_.param("var_low", var_low_);
    nh_private_.param("var_high", var_high_);

    nh_private_.param("scale", scale_);
    nh_private_.param("close_range", close_range_);
  }

private:
  inline void update(const versavis::LidarLiteMicro::ConstPtr &in) override {
    out_.range.header.seq = in->number;
    out_.range.header.stamp = in->time.data;
    out_.range.range = scale_ * in->range;
    out_.signal_strength = in->signal_strength;
    out_.variance = out_.range.range < close_range_ ? var_low_ : var_high_;
  }

  double scale_ = 0.0;
  double close_range_ = 0.0;
  double var_low_ = -1.0;
  double var_high_ = -1.0;
};

} // namespace versavis

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_lite_transform");
  versavis::LidarLiteTransform tf();
  ros::spin();
  return 0;
}
