#ifndef VERSAVIS_IMPL_TOPIC_TRANSFORM_IMPL_H_
#define VERSAVIS_IMPL_TOPIC_TRANSFORM_IMPL_H_

namespace versavis {

template <class MsgIn, class MsgOut>
TopicTransform<MsgIn, MsgOut>::TopicTransform() : nh_private_("~") {
  // Subscribe.
  int queue_size = 1000;
  nh_private_.getParam("queue_size", queue_size);
  sub_ = nh_.subscribe("in", queue_size, &TopicTransform::callback, this);

  // Publish.
  bool latch = true;
  nh_private_.getParam("latch", latch);
  pub_ = nh_.advertise<MsgOut>("out", queue_size, latch);
}

template <class MsgIn, class MsgOut>
void TopicTransform<MsgIn, MsgOut>::callback(
    const typename MsgIn::ConstPtr &in) {
  update(in);
  pub_.publish(out_);
}

} // namespace versavis

#endif // VERSAVIS_IMPL_TOPIC_TRANSFORM_IMPL_H_
