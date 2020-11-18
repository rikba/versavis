#include "versavis/MagneticMicro.h"
#include "versavis/topic_transform.h"

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

namespace versavis {
class MagTransform : public TopicTransform<versavis::MagneticMicro,
                                           sensor_msgs::MagneticField> {
public:
  inline MagTransform() {
    nh_private_.param("frame_id", out_.header.frame_id);

    nh_private_.param("cov", out_.magnetic_field_covariance[0], -1.0);
    nh_private_.param("cov", out_.magnetic_field_covariance[4]);
    nh_private_.param("cov", out_.magnetic_field_covariance[8]);

    nh_private_.param("scale", scale_);
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

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_transform");
  versavis::MagTransform tf;
  ros::spin();
  return 0;
}
