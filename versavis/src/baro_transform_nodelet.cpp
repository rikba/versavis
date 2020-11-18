#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/baro_transform.h"

namespace versavis {

class BaroTransformNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      tf_ = std::make_shared<BaroTransform>(getNodeHandle(),
                                            getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<BaroTransform> tf_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::BaroTransformNodelet, nodelet::Nodelet)
