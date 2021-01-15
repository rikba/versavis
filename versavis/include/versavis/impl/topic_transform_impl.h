#ifndef VERSAVIS_IMPL_TOPIC_TRANSFORM_IMPL_H_
#define VERSAVIS_IMPL_TOPIC_TRANSFORM_IMPL_H_

namespace versavis {

template <class MsgIn, class MsgOut>
TopicTransform<MsgIn, MsgOut>::TopicTransform(const ros::NodeHandle &nh,
                                              const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Subscribe.
  int queue_size = 1000;
  nh_private_.getParam("queue_size", queue_size);
  sub_ = nh_.subscribe("in", queue_size, &TopicTransform::callback, this);

  // Publish.
  bool latch = true;
  nh_private_.getParam("latch", latch);
  pub_ = nh_.advertise<MsgOut>("out", queue_size, latch);

  ROS_INFO("Topic transformer subscribing to: %s Publishing to: %s.",
           sub_.getTopic().c_str(), pub_.getTopic().c_str());
}

template <class MsgIn, class MsgOut>
void TopicTransform<MsgIn, MsgOut>::callback(
    const typename MsgIn::ConstPtr &in) {
  if (in->time.data != ros::Time()) {
    update(in);
    typename MsgOut::ConstPtr out_ptr(new MsgOut(out_));
    pub_.publish(out_ptr);
  }
}

} // namespace versavis

#endif // VERSAVIS_IMPL_TOPIC_TRANSFORM_IMPL_H_
