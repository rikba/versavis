#include "sensors/ExternalEventSynced.h"

ExternalEventSynced::ExternalEventSynced(ros::NodeHandle *nh,
                                         TimerSynced *timer, bool invert)
    : nh_(nh), timer_(timer) {
  // Setup pin to capture external event.
  if (timer_) {
    timer_->setupExternalEvent(invert);
  }
}

void ExternalEventSynced::setupRos(char *frame_id, char *event_topic) {
  if (nh_) {
    // Create static ROS msg.
    static std_msgs::Header msg;

    // Assign topic pointer.
    msg_ = &msg;

    // Create static ROS publisher.
    static ros::Publisher pub(event_topic, msg_);

    // Assign publisher pointers.
    publisher_ = &pub;

    // Advertise.
    nh_->advertise(*publisher_);
  }

  // Initialize.
  if (msg_) {
    msg_->frame_id = frame_id;
  }
}

bool ExternalEventSynced::publish() {
  bool new_measurement = false;

  // Obtain new stamp.
  if (timer_ && msg_ && timer_->getTimeLastTrigger(&msg_->stamp, &msg_->seq)) {
    if (publisher_) {
      publisher_->publish(msg_);
    }
    new_measurement = true;
  }

  return new_measurement;
}
