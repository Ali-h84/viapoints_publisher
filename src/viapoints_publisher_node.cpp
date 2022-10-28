#include <ros/ros.h>
#include <viapoints_publisher/viapoints_publisher.h>

using namespace viapoints_publisher;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ViaPointsPublisher viapoints_publisher(nh);
  ros::spin();
  return 0;
}
