#ifndef SCOUT_ODOM_HPP
#define SCOUT_ODOM_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace scout_odom {
	class ScoutOdom {
		public:
			ScoutOdom(ros::NodeHandle *n, std::string odom_topic = "/odom", 
					std::string cmd_topic = "/cmd_vel");
			
			void SetupSubscription();
		
		private:
			std::string odom_topic_;
			std::string cmd_topic_;
			
			ros::Time t_last_change_;

			double x_;
			double y_;
			double z_;

			double theta_;
			double rotation_axis_[3];
			
			ros::NodeHandle *n_;
			
			ros::Publisher odom_pub_;
			ros::Subscriber cmd_sub_;

			tf2_ros::Buffer *tf_buffer_;
  			tf2_ros::TransformListener *tf_listener_;


			ros::Timer timer_;
			void GetInitialPosition();
			void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
	};
} // scout_odom namespace

#endif /* SCOUT_ODOM_TOPIC */
