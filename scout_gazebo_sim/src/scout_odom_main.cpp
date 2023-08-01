#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include "scout_gazebo/scout_odom.hpp"

using namespace scout_odom;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters
  std::string odom_topic;
  private_node.param<std::string>("robot_namespace", odom_topic,
                                  std::string("/odom"));

  std::string cmd_topic;
  private_node.param<std::string>("cmd_topic", cmd_topic,
                                  std::string("/cmd_vel"));
  
  ROS_INFO("Odom uses: %s to generate: %s", 
    cmd_topic.c_str(), odom_topic.c_str());

  ScoutOdom scout_odom(&node, odom_topic, cmd_topic);
  scout_odom.SetupSubscription();

  ros::spin();

  return 0;
}