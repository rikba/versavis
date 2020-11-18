#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/mag_transform.h"

namespace versavis {

class MagTransformNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      tf_ = std::make_shared<MagTransform>(getNodeHandle(),
                                           getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<MagTransform> tf_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::MagTransformNodelet, nodelet::Nodelet)
