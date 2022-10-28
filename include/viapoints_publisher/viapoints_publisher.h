#ifndef VIAPOINTS_PUBLISHER_H
#define VIAPOINTS_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

namespace viapoints_publisher
{

class ViaPointsPublisher
{
public:
  enum PointType {UNDEFINED, TRANSLATION, ROTATION, ARC};

  ViaPointsPublisher(ros::NodeHandle &nh);
  void pathCallback(const nav_msgs::PathConstPtr &path_msg);
  void findViaPoints(
      const nav_msgs::PathConstPtr &path_msg,
      nav_msgs::Path &via_points
  );
  PointType getPointType(const double translation, const double rotation);
  void updateParams();
  bool getRobotPose(
      const std::string global_frame_id,
      geometry_msgs::PoseStamped &global_pose,
      const double timeout=3.0
  );

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Publisher viapoints_pub_;
  ros::Subscriber path_sub_;

  const std::string name_;

  std::string robot_frame_id_;
  double translation_threshold_;
  double rotation_threshold_;
  double max_viapoint_separation_;
  double max_lookahead_distance_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
}; // endclass ViaPointsPublisher

} // endnamespace viapoints_publisher

#endif // VIAPOINTS_PUBLISHER_H
