#include "scout_gazebo/scout_odom.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace scout_odom {

	ScoutOdom::ScoutOdom(ros::NodeHandle *n, std::string odom_topic,
			std::string cmd_topic){
		n_ = n;	
		odom_topic_ = odom_topic;
		cmd_topic_ = cmd_topic;
	}

	void ScoutOdom::GetInitialPosition(){
		geometry_msgs::TransformStamped transformStamped;
			
		try{
			transformStamped = tf_buffer_->lookupTransform("map", "odom",
			                         ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			return;
		}
			
		t_last_change_ = transformStamped.header.stamp;

		x_ = transformStamped.transform.translation.x;
		y_ = transformStamped.transform.translation.y;
		z_ = transformStamped.transform.translation.z;
			
		theta_ = 2*acos(transformStamped.transform.rotation.x);
			
		double x_squared = transformStamped.transform.rotation.x * 
			transformStamped.transform.rotation.x;

		rotation_axis_[0] = transformStamped.transform.rotation.y / 
			sqrt(1 - x_squared);
		rotation_axis_[1] = transformStamped.transform.rotation.z / 
			sqrt(1 - x_squared);
		rotation_axis_[2] = transformStamped.transform.rotation.w / 
			sqrt(1 - x_squared);
	}

	void ScoutOdom::SetupSubscription() {
		cmd_sub_ = n_->subscribe<geometry_msgs::Twist>(
    		cmd_topic_, 10, &ScoutOdom::TwistCmdCallback, this);
		
		odom_pub_ = n_->advertise<nav_msgs::Odometry>(odom_topic_, 30);
			
		tf_buffer_ = new tf2_ros::Buffer();
  		tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);

		//ros::Timer timer_ = n_->createTimer(ros::Duration(0.1), 
		//		&ScoutOdom::GetInitialPosition, this);

		ScoutOdom::GetInitialPosition(); 
	}

	void ScoutOdom::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
		ros::Time current_time = ros::Time::now();
		double driving_vel = msg->linear.x;
		double steering_vel = msg->angular.z;
		
		double dt = (current_time - t_last_change_).toSec();
		x_ += driving_vel * cos(theta_) * dt;
		y_ += driving_vel * sin(theta_) * dt;
		theta_ += steering_vel * dt;
		
		// TODO: how to work with quaternions
		// double q_x = cos(theta_/2);
		// double q_y = rotation_axis_[0] * sin(theta_/2);
		// double q_z = rotation_axis_[1] * sin(theta_/2);
		// double q_w = rotation_axis_[2] * sin(theta_/2);

		// double q_norm = sqrt(q_x*q_x + q_y*q_y + q_z*q_z + q_w*q_w);
		// q_x /= q_norm;
		// q_y /= q_norm;
		// q_z /= q_norm;
		// q_w /= q_norm;

		tf2::Quaternion odom_quat;
		odom_quat.setRPY(0, 0, theta_);		

		nav_msgs::Odometry odometry;
		odometry.header.stamp = current_time;
		odometry.header.frame_id = "odom";
		odometry.child_frame_id = "map";

		odometry.pose.pose.position.x = x_;
		odometry.pose.pose.position.y = y_;
		odometry.pose.pose.position.z = z_;

		odometry.pose.pose.orientation = tf2::toMsg(odom_quat);

		odometry.twist.twist = *msg;

		odom_pub_.publish(odometry);	
		
		t_last_change_ = current_time;
	}
} // namespace scout_odom
