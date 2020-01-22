#include "ExternalEvent.h"

ExternalEvent::ExternalEvent(ros::NodeHandle *nh, const String &topic)
    : nh_(nh), topic_(topic) {
  if (nh_ == nullptr) {
    error((topic_ + " (ExternalEvent.cpp): The node handle is not available.")
              .c_str(),
          49);
  }
}
