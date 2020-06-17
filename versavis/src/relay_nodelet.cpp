#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/relay.h"

namespace versavis {

class RelayNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      relay_ = std::make_shared<Relay>(getNodeHandle(), getPrivateNodeHandle());
      relay_->initialize();
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<Relay> relay_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::RelayNodelet, nodelet::Nodelet)
