#include "versavis/ImuMicro.h"
#include "versavis/topic_transform.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace versavis {
class ImuTransform
    : public TopicTransform<versavis::ImuMicro, sensor_msgs::Imu> {
  inline ImuTransform() {
    nh_private_.param("frame_id", out_.header.frame_id);

    nh_private_.param("orientation_cov", out_.orientation_covariance[0], -1.0);
    nh_private_.param("orientation_cov", out_.orientation_covariance[4]);
    nh_private_.param("orientation_cov", out_.orientation_covariance[8]);

    nh_private_.param("gyro_cov", out_.angular_velocity_covariance[0], -1.0);
    nh_private_.param("gyro_cov", out_.angular_velocity_covariance[4]);
    nh_private_.param("gyro_cov", out_.angular_velocity_covariance[8]);

    nh_private_.param("acc_cov", out_.linear_acceleration_covariance[0], -1.0);
    nh_private_.param("acc_cov", out_.linear_acceleration_covariance[4]);
    nh_private_.param("acc_cov", out_.linear_acceleration_covariance[8]);

    nh_private_.param("acc_scale", acc_scale_);
    nh_private_.param("gyro_scale", gyro_scale_);
  }

private:
  inline void update(const versavis::ImuMicro::ConstPtr &in) override {
    out_.header.seq = in->number;
    out_.header.stamp = in->time.data;

    out_.angular_velocity.x = gyro_scale_ * in->gx;
    out_.angular_velocity.y = gyro_scale_ * in->gy;
    out_.angular_velocity.z = gyro_scale_ * in->gz;

    out_.linear_acceleration.x = acc_scale_ * in->ax;
    out_.linear_acceleration.y = acc_scale_ * in->ay;
    out_.linear_acceleration.z = acc_scale_ * in->az;
  }

  double acc_scale_ = 0.0;
  double gyro_scale_ = 0.0;
};

} // namespace versavis

// Standard C++ entry point
int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_transform");
  versavis::ImuTransform tf();
  ros::spin();
  return 0;
}
