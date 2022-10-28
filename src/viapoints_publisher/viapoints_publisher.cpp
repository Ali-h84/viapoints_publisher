#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <array>
#include <cmath>

#include <viapoints_publisher/viapoints_publisher.h>

namespace viapoints_publisher
{

// utility function prototypes

/**
 * wrap angle in range [-pi, pi]
 * https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
 */
double angleWrap(double x);

/**
 * gets relative translation
 */
double getRelativeTranslation(
    const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2
);

/**
 * gets relative pose rotation
 */
double getRelativeRotation(
    const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2
);

/**
 * gets relative pose, (translation, rotation) array
 */
std::array<double, 2> getRelativePose(
    const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2
);

ViaPointsPublisher::ViaPointsPublisher(ros::NodeHandle &nh)
  : nh_(nh), local_nh_("~"), name_("viapoints_publisher"),
    tf_buffer_(), tf_listener_(tf_buffer_)
{
  viapoints_pub_= nh_.advertise<nav_msgs::Path>("via_points", 10, true);
  path_sub_ = nh_.subscribe("global_path", 1, &ViaPointsPublisher::pathCallback, this);

  updateParams();
}

void ViaPointsPublisher::updateParams()
{
  local_nh_.param<double>("translation_threshold", translation_threshold_, 1e-2);
  local_nh_.param<double>("rotation_threshold", rotation_threshold_, 1e-2);
  local_nh_.param<double>("max_viapoint_separation", max_viapoint_separation_, 0.5);

  double teb_lookahead_dist;
  if (!nh_.getParam("/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist", teb_lookahead_dist))
  {
    ROS_ERROR("TEB lookahead distance parameter not found");
  }

  nh_.param<std::string>(
      "/move_base/local_costmap/robot_base_frame", robot_frame_id_, "base_link"
  );

  double local_costmap_width;
  double local_costmap_height;
  nh_.param<double>("/move_base/local_costmap/width", local_costmap_width, 5.0);
  nh_.param<double>("/move_base/local_costmap/height", local_costmap_height, 5.0);

  max_lookahead_distance_ = std::max(
      teb_lookahead_dist,
      std::max(local_costmap_width/2.0, local_costmap_height/2.0)
  );

  ROS_INFO_NAMED(
      name_,
      "Parameters \n"
      "translation_threshold: %f\n"
      "rotation_threshold: %f\n"
      "max_viapoint_separation: %f\n"
      "max_lookahead_distance: %f\n"
      "robot_frame_id: %s\n",
      translation_threshold_, rotation_threshold_,
      max_viapoint_separation_, max_lookahead_distance_,
      robot_frame_id_.c_str()
  );
}

bool ViaPointsPublisher::getRobotPose(
    const std::string global_frame_id,
    geometry_msgs::PoseStamped &global_pose,
    const double timeout
)
{
  geometry_msgs::PoseStamped local_pose;

  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  tf2::toMsg(tf2::Transform::getIdentity(), local_pose.pose);
  local_pose.header.frame_id = robot_frame_id_;
  local_pose.header.stamp = ros::Time::now();;

  try
  {
    global_pose = tf_buffer_.transform(
        local_pose, global_frame_id, ros::Duration(timeout)
    );
    return true;
  }
  catch (tf2::LookupException & ex)
  {
    ROS_ERROR_NAMED(
      name_,
      "No Transform available Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ConnectivityException & ex)
  {
    ROS_ERROR_NAMED(
      name_,
      "Connectivity Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ExtrapolationException & ex)
  {
    ROS_ERROR_NAMED(
      name_,
      "Extrapolation Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::TimeoutException & ex)
  {
    ROS_ERROR_NAMED(
      name_,
      "Transform timeout with tolerance: %.4f", timeout);
  }
  catch (tf2::TransformException & ex)
  {
    ROS_ERROR_NAMED(
      name_, "Failed to transform from %s to %s",
      global_frame_id.c_str(), robot_frame_id_.c_str());
  }

  return false;
}

void ViaPointsPublisher::pathCallback(const nav_msgs::PathConstPtr &path_msg)
{
  // ROS_INFO_THROTTLE_NAMED(1.0, name_, "path received");
  nav_msgs::Path via_points;
  via_points.header = path_msg->header;
  findViaPoints(path_msg, via_points);
  viapoints_pub_.publish(via_points);
}

void ViaPointsPublisher::findViaPoints(
    const nav_msgs::PathConstPtr &path_msg,
    nav_msgs::Path &via_points
)
{
  assert(path_msg);
  via_points.poses.clear();

  PointType previous_point_type = UNDEFINED;
  geometry_msgs::Pose last_viapoint_pose = path_msg->poses.front().pose;
  geometry_msgs::PoseStamped robot_pose;
  auto is_robot_pose_correct = getRobotPose(
      path_msg->header.frame_id, robot_pose
  );

  if (!is_robot_pose_correct)
  {
    ROS_ERROR_NAMED(
        name_, "robot pose not correct, publishing empty via-points"
    );
    return;
  }

  for (size_t point_idx = 0; point_idx < path_msg->poses.size()-1; point_idx++)
  {
    const auto current_pose = path_msg->poses[point_idx].pose;
    const auto next_pose = path_msg->poses[point_idx+1].pose;
    const auto translation = getRelativeTranslation(current_pose, next_pose);
    const auto rotation = getRelativeRotation(current_pose, next_pose);
    const auto dist_to_robot = getRelativeTranslation(robot_pose.pose, current_pose);

    const auto current_point_type = getPointType(translation, rotation);
    bool is_viapoint = false;

    if (dist_to_robot >= max_lookahead_distance_)
    {
      is_viapoint = false;
    }
    else if (current_point_type != previous_point_type || current_point_type == ROTATION)
    {
      // always add points of discontinuity
      // always add rotation points to avoid local planner from taking an arc motion
      is_viapoint = true;
    }
    else
    {
      // add via-points at certain distances
      const auto last_viapoint_dist = getRelativeTranslation(
          last_viapoint_pose, current_pose
      );
      is_viapoint = last_viapoint_dist >= max_viapoint_separation_;
    }

    if (is_viapoint)
    {
      via_points.poses.push_back(path_msg->poses[point_idx]);
      last_viapoint_pose = via_points.poses.back().pose;
    }

    previous_point_type = current_point_type;
  }

  // this causes the local planner to oscillate at the end
  // via_points.poses.push_back(path_msg->poses.back());
}

ViaPointsPublisher::PointType ViaPointsPublisher::getPointType(
    const double translation, const double rotation
)
{
  bool is_translation = translation > translation_threshold_;
  bool is_rotation = rotation > rotation_threshold_;

  PointType current_point_type = UNDEFINED;
  if (is_translation && !is_rotation)
  {
    current_point_type = TRANSLATION;
  }
  else if (!is_translation && is_rotation)
  {
    current_point_type = ROTATION;
  }
  else if (is_translation && is_rotation)
  {
    current_point_type = ARC;
  }

  return current_point_type;
}

double angleWrap(double x)
{
  constexpr auto M_PI_twice = 2 * M_PI;
  x = std::fmod(x + M_PI, M_PI_twice);
  if (x < 0)
      x += M_PI_twice;
  return x - M_PI;
}

double getRelativeTranslation(
    const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2
)
{
  return std::hypot(
      pose2.position.x - pose1.position.x,
      pose2.position.y - pose1.position.y
  );
}

double getRelativeRotation(
    const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2
)
{
  const auto rotation1 = tf2::getYaw(pose1.orientation);
  const auto rotation2 = tf2::getYaw(pose2.orientation);
  return angleWrap(rotation2 - rotation1);
}

std::array<double, 2> getRelativePose(
    const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2
)
{
  auto translation = getRelativeTranslation(pose1, pose2);
  auto rotation = getRelativeRotation(pose1, pose2);
  return {translation, rotation};
}

} // endnamespace viapoints_publisher
