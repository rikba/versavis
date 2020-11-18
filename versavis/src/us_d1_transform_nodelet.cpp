#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/us_d1_transform.h"

namespace versavis {

class UsD1TransformNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      tf_ = std::make_shared<UsD1Transform>(getNodeHandle(),
                                            getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<UsD1Transform> tf_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::UsD1TransformNodelet, nodelet::Nodelet)
