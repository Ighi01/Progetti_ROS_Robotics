#include "ros/ros.h"
#include "robo1/MessageOdometry.h"
#include "robo1/MessageVelocity.h"
#include <cmath>
#include "robo1/ResetOdometryToPose.h"
#include <robo1/methodConfig.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include <sstream>
#include <bitset>
#include "nav_msgs/Odometry.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

enum integration_methods {Euler, RK};
integration_methods integration_method = Euler;
double x,y,theta;
ros::Time ts;
ros::Publisher pub_odom;
nav_msgs::Odometry msg_odom;
geometry_msgs::TransformStamped transformStamped;
bool is_first;

bool reset_odometry_to_pose(robo1::ResetOdometryToPose::Request &req, robo1::ResetOdometryToPose::Response &res){
  res.old_x= x;
  res.old_y= y;
  res.old_theta= theta;

  x= req.new_x;
  y = req.new_y;
  theta = req.new_theta;
  is_first=true;

  return true;
}


void change_method(integration_methods* integration_method, robo1::methodConfig &config, uint32_t level){
  
  
  switch(config.integration_method){
    case 0: {*integration_method = Euler; }
        break;
    case 1: {*integration_method = RK; }
        break;
  }
  
}


void calculate_odometries(const geometry_msgs::TwistStamped::ConstPtr& msg){      
	
	if(!is_first){
	double delta_ts;        
	delta_ts = (msg -> header.stamp - ts).toSec();
	ts = msg -> header.stamp;

	theta= theta + msg -> twist.angular.z *delta_ts;

	if(integration_method == Euler){

		x = x + delta_ts * (msg -> twist.linear.x * cos(theta) -  msg -> twist.linear.y * sin(theta) );
		y = y + delta_ts * (msg -> twist.linear.x * sin(theta) +  msg -> twist.linear.y * cos(theta) );
	}
	else
	{ 
		x = x + delta_ts *( cos(msg -> twist.angular.z * delta_ts / 2) * (msg -> twist.linear.x * cos(theta) - msg -> twist.linear.y * sin(theta) ) - 
				    sin(msg -> twist.angular.z * delta_ts / 2) * (msg -> twist.linear.x * sin(theta) + msg -> twist.linear.y * cos(theta) ) );
	
		y = y + delta_ts *( cos(msg -> twist.angular.z * delta_ts / 2) * (msg -> twist.linear.x * sin(theta) + msg -> twist.linear.y * cos(theta) ) + 
				    sin(msg -> twist.angular.z * delta_ts / 2) * (msg -> twist.linear.x * cos(theta) - msg -> twist.linear.y * sin(theta) ) );
	}

        static tf2_ros::TransformBroadcaster transform_broadcaster; 

 	transformStamped.header.stamp = ts;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        transform_broadcaster.sendTransform(transformStamped);

	msg_odom.header.stamp = ts;
	msg_odom.header.frame_id = "odom";

	msg_odom.pose.pose.position.x = x;
	msg_odom.pose.pose.position.y = y;
	msg_odom.pose.pose.position.z = 0.0;
	msg_odom.pose.pose.orientation.x = q.x();
	msg_odom.pose.pose.orientation.y = q.y();
	msg_odom.pose.pose.orientation.z = q.z();
	msg_odom.pose.pose.orientation.w = q.w();

	msg_odom.child_frame_id = "base_link";
	msg_odom.twist.twist.linear.x = msg -> twist.linear.x;
	msg_odom.twist.twist.linear.y = msg -> twist.linear.y;
	msg_odom.twist.twist.linear.z = msg -> twist.linear.z;

	msg_odom.twist.twist.angular.x = msg -> twist.angular.x;
	msg_odom.twist.twist.angular.y = msg -> twist.angular.y;
	msg_odom.twist.twist.angular.z = msg -> twist.angular.z;

	pub_odom.publish(msg_odom);
	}
	ts = msg -> header.stamp;
	is_first=false;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "pub_odometry");
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf2_ros::TransformBroadcaster transform_broadcaster;

	
	ros::ServiceServer reset_odometry_to_pose_service =
		n.advertiseService<robo1::ResetOdometryToPose::Request,
						   robo1::ResetOdometryToPose::Response>("reset", reset_odometry_to_pose);
   

	dynamic_reconfigure::Server<robo1::methodConfig> dynServer;
	dynamic_reconfigure::Server<robo1::methodConfig>::CallbackType f;
	f = boost::bind(&change_method, &integration_method,_1, _2);
	dynServer.setCallback(f);
	
	n.getParam("initialPoseX", x);
	n.getParam("initialPoseY", y);
	n.getParam("initialPoseTheta", theta);
	is_first=true;

	pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1000);
        sub = n.subscribe("/cmd_vel", 1000, calculate_odometries);

        ros::spin();
	return 0;
}
