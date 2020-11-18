#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/lidar_lite_transform.h"

namespace versavis {

class LidarLiteTransformNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      tf_ = std::make_shared<LidarLiteTransform>(getNodeHandle(),
                                                 getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<LidarLiteTransform> tf_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::LidarLiteTransformNodelet, nodelet::Nodelet)
