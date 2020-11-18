#include "versavis/PressureMicro.h"
#include "versavis/topic_transform.h"

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

namespace versavis {
class BaroTransform : public TopicTransform<versavis::PressureMicro,
                                            sensor_msgs::FluidPressure> {
  inline BaroTransform() {
    nh_private_.param("frame_id", out_.header.frame_id);
    nh_private_.param("var", out_.variance);
    nh_private_.param("scale", scale_);
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

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "baro_transform");
  versavis::BaroTransform tf();
  ros::spin();
  return 0;
}
