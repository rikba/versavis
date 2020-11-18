#include "versavis/TemperatureMicro.h"
#include "versavis/topic_transform.h"

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>

namespace versavis {
class TempTransform : public TopicTransform<versavis::TemperatureMicro,
                                            sensor_msgs::Temperature> {
public:
  inline TempTransform() {
    nh_private_.param("frame_id", out_.header.frame_id);
    nh_private_.param("var", out_.variance);
    nh_private_.param("scale", scale_);
    nh_private_.param("offset", offset_);
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

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "temp_transform");
  versavis::TempTransform tf;
  ros::spin();
  return 0;
}
