#ifndef VERSAVIS_IMU_TRANSFORM_H_
#define VERSAVIS_IMU_TRANSFORM_H_

#include "versavis/ImuMicro.h"
#include "versavis/topic_transform.h"

#include <sensor_msgs/Imu.h>

namespace versavis {
class ImuTransform
    : public TopicTransform<versavis::ImuMicro, sensor_msgs::Imu> {
public:
  inline ImuTransform(const ros::NodeHandle &nh,
                      const ros::NodeHandle &nh_private)
      : TopicTransform(nh, nh_private) {
    nh_private_.getParam("frame_id", out_.header.frame_id);

    out_.orientation_covariance[0] = -1.0;
    nh_private_.getParam("orientation_cov", out_.orientation_covariance[0]);
    nh_private_.getParam("orientation_cov", out_.orientation_covariance[4]);
    nh_private_.getParam("orientation_cov", out_.orientation_covariance[8]);

    out_.angular_velocity_covariance[0] = -1.0;
    nh_private_.getParam("gyro_cov", out_.angular_velocity_covariance[0]);
    nh_private_.getParam("gyro_cov", out_.angular_velocity_covariance[4]);
    nh_private_.getParam("gyro_cov", out_.angular_velocity_covariance[8]);

    out_.linear_acceleration_covariance[0] = -1.0;
    nh_private_.getParam("acc_cov", out_.linear_acceleration_covariance[0]);
    nh_private_.getParam("acc_cov", out_.linear_acceleration_covariance[4]);
    nh_private_.getParam("acc_cov", out_.linear_acceleration_covariance[8]);

    nh_private_.getParam("acc_scale", acc_scale_);
    ROS_INFO("Accelerometer scale: %.6f", acc_scale_);
    nh_private_.getParam("gyro_scale", gyro_scale_);
    ROS_INFO("Gyro scale: %.6f", gyro_scale_);
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

#endif // VERSAVIS_IMU_TRANSFORM_H_
