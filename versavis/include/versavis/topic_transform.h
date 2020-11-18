#ifndef VERSAVIS_TOPIC_TRANSFORM_H_
#define VERSAVIS_TOPIC_TRANSFORM_H_

#include <ros/ros.h>

namespace versavis {

template <class MsgIn, class MsgOut> class TopicTransform {
public:
  TopicTransform();

protected:
  virtual void update(const typename MsgIn::ConstPtr &in) = 0;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber sub_;
  ros::Publisher pub_;

  MsgOut out_;

private:
  void callback(const typename MsgIn::ConstPtr &in);
};

} // namespace versavis

#include "versavis/impl/topic_transform_impl.h"

#endif // VERSAVIS_TOPIC_TRANSFORM_H_
