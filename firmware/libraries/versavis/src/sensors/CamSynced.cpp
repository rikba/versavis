#include "sensors/CamSynced.h"

CamSynced::CamSynced(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz, const bool invert)
    : SensorSynced(nh, timer) {
  // Setup trigger.
  if (timer) {
    timer->setupMfrq(rate_hz, invert);
  }
}

void CamSynced::setupRos(char *frame_id, char *rate_topic, char *seq_topic,
                         char *img_topic) {
  static ros::Subscriber<std_msgs::UInt16, SensorSynced> rate_sub(
      rate_topic, &CamSynced::changeRateCb, this);
  SensorSynced::setupRos(rate_sub);
  if (nh_) {
    // Create static ROS message.
    static std_msgs::Header img_msg;
    img_msg_ = &img_msg;

    // Create static ROS publisher.
    static ros::Publisher pub(img_topic, img_msg_);
    publisher_ = &pub;

    static ros::Subscriber<std_msgs::UInt32, CamSynced> seq_sub(
        seq_topic, &CamSynced::setSeqCb, this);

    // Advertise.
    nh_->advertise(pub);
    nh_->subscribe(seq_sub); // This needs to be moved outside of class prolly.

    // Initialize
    img_msg_->frame_id = frame_id;
  }
}

void CamSynced::setSeqCb(const std_msgs::UInt32 &seq_msg) {
  if (timer_) {
    timer_->setTriggerStateNum(seq_msg.data);
  }
  if (nh_) {
    nh_->loginfo("Setting trigger header sequence.");
  }
}