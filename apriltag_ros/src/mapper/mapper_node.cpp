#include "apriltag_ros/mapper_node.h"

namespace apriltag_ros {

void MapperNode::TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_msg) {
  ROS_INFO_THROTTLE(2, "In tags cb");
}

}  // namespace apriltag_ros