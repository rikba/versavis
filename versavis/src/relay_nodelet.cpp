#include <thread>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "versavis/relay.h"

namespace versavis {

class RelayNodelet : public nodelet::Nodelet {
public:
  inline ~RelayNodelet() {
    if (startup_thread_.joinable())
      startup_thread_.join();
  }

private:
  virtual void onInit() {
    try {
      relay_ = std::make_shared<Relay>(getNodeHandle(), getPrivateNodeHandle());
      startup_thread_ = std::thread(&Relay::initialize, relay_);
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<Relay> relay_;
  std::thread startup_thread_;
};
} // namespace versavis

PLUGINLIB_EXPORT_CLASS(versavis::RelayNodelet, nodelet::Nodelet)
