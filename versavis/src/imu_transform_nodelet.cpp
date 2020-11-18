#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/imu_transform.h"

namespace versavis {

class ImuTransformNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      tf_ = std::make_shared<ImuTransform>(getNodeHandle(),
                                           getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<ImuTransform> tf_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::ImuTransformNodelet, nodelet::Nodelet)
