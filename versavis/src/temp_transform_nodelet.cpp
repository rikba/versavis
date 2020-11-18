#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/temp_transform.h"

namespace versavis {

class TempTransformNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      tf_ = std::make_shared<TempTransform>(getNodeHandle(),
                                            getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<TempTransform> tf_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::TempTransformNodelet, nodelet::Nodelet)
