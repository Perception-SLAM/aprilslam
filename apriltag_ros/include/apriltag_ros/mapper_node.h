#ifndef APRILTAG_ROS_MAPPER_NODE_H_
#define APRILTAG_ROS_MAPPER_NODE_H_

#include <ros/ros.h>
#include <apriltag_ros/Apriltags.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include "apriltag_ros/visualizer.h"
#include <tf2_ros/transform_broadcaster.h>

#include "apriltag_ros/mapper.h"
#include "apriltag_ros/tag_map.h"

namespace apriltag_ros {

class MapperNode {
 public:
  MapperNode(const ros::NodeHandle& nh, const std::string& frame_id)
      : nh_(nh),
        sub_tags_(nh_.subscribe("apriltags", 1, &MapperNode::TagsCb, this)),
        sub_cinfo_(nh_.subscribe("camera_info", 1, &MapperNode::CinfoCb, this)),
        frame_id_(frame_id),
        mapper_(0.04, 1),
        tag_viz_(nh, "apriltags_map") {
    		tag_viz_.SetColor(apriltag_ros::GREEN);
    		tag_viz_.SetAlpha(0.75);
  	}

  bool GetGoodTags(const std::vector<apriltag_ros::Apriltag> tags_c,
                   std::vector<apriltag_ros::Apriltag>* tags_c_good);

 private:
  void TagsCb(const apriltag_ros::ApriltagsConstPtr& tags_c_msg);
  void CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_tags_;
  ros::Subscriber sub_cinfo_;
  std::string frame_id_;
  apriltag_ros::TagMap map_;
  apriltag_ros::Mapper mapper_;
  apriltag_ros::ApriltagVisualizer tag_viz_;
  image_geometry::PinholeCameraModel model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_MAPPER_NODE_H_
