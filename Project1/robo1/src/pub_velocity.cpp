#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "robo1/MessageOdometry.h"
#include "robo1/MessageVelocity.h"
#include <boost/shared_ptr.hpp>
#include <cmath>
#include "robo1/CalibrationMessage.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

double WHEEL_POSITION_ALONG_X, WHEEL_POSITION_ALONG_Y, WHEEL_RADIUS;
ros::Time ts;
ros::Publisher pub;

void calculate_velocities(const robo1::CalibrationMessage::ConstPtr& msg)  
	{
		
                /*WHEEL_POSITION_ALONG_X= msg-> L;
                WHEEL_POSITION_ALONG_Y= msg-> W;
                WHEEL_RADIUS= msg-> R;*/

		double w =  (WHEEL_RADIUS * (msg -> u3 - msg ->u0)) / (2* (WHEEL_POSITION_ALONG_X + WHEEL_POSITION_ALONG_Y));
		double vx = 0.5 * WHEEL_RADIUS * (msg ->u1 + msg ->u0);
		double vy = 0.5 * WHEEL_RADIUS * (msg ->u2 - msg ->u0);

		geometry_msgs::TwistStamped msg_vel;

		msg_vel.header.stamp = msg -> header.stamp;
		msg_vel.header.frame_id = "base_link";

		msg_vel.twist.linear.x = vx;
		msg_vel.twist.linear.y = vy;
		msg_vel.twist.linear.z = 0.0;
		msg_vel.twist.angular.x = 0.0;
		msg_vel.twist.angular.y = 0.0;
		msg_vel.twist.angular.z = w;
		pub.publish(msg_vel);

	}

	int main(int argc, char** argv){
		
                ros::init(argc, argv, "pub_velocity");
		ros::NodeHandle n;
                ros::Subscriber sub;

		n.getParam("robotDimensionL", WHEEL_POSITION_ALONG_X); 
   		n.getParam("robotDimensionW", WHEEL_POSITION_ALONG_Y); 
   		n.getParam("robotDimensionR", WHEEL_RADIUS); 
    	
                pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
		sub = n.subscribe<robo1::CalibrationMessage>("/calibration_msg",1000 ,calculate_velocities);
		ros::spin();
		return 0;
	}
