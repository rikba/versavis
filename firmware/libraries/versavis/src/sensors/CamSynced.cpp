#include "sensors/CamSynced.h"

CamSynced::CamSynced(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz, const MeasurementState &meas_state)
    : SensorSynced(nh, timer) {
  // Setup trigger.
  if (timer) {
    timer->setMeasurementState(meas_state);
    timer->setupMfrq(rate_hz);
  }
}

CamSynced::CamSynced(ros::NodeHandle *nh, TimerSynced *timer,
                     const uint16_t rate_hz, const bool invert)
    : CamSynced(nh, timer, rate_hz,
                {SensorInterface::kSingleCapture,
                 SensorInterface::kSingleCapture,
                 SensorInterface::kSingleCapture, invert, false, false}) {}

void CamSynced::setupRos(const char *frame_id, const char *rate_topic,
                         const char *seq_topic, const char *img_topic) {
  static ros::Subscriber<std_msgs::UInt16, SensorSynced> rate_sub(
      rate_topic, &CamSynced::changeRateCb, this);
  SensorSynced::setupRos(rate_sub);

  // Create static ROS message.
  static std_msgs::Header img_msg;
  img_msg_ = &img_msg;

  if (nh_) {

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
    timer_->setLatestMeasurementNum(seq_msg.data);
  }
  if (nh_) {
    nh_->loginfo("Setting image header sequence.");
  }
}
