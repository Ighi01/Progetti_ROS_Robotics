#include "ros/ros.h"
#include "robo1/MessageOdometry.h"
#include "robo1/MessageVelocity.h"
#include "robo1/WheelsRPM.h"
#include <cmath>
#include <robo1/parametersConfig.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/String.h"
#include <sstream>
#include <bitset>
#include "nav_msgs/Odometry.h"
#include "robo1/CalibrationMessage.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

double WHEEL_POSITION_ALONG_X, WHEEL_POSITION_ALONG_Y, WHEEL_RADIUS, GEAR_RATIO;
ros::Publisher pub_rpm;

void calculate_RPM(const geometry_msgs::TwistStamped::ConstPtr& msg){

    robo1::WheelsRPM msg_RPM;
    
    msg_RPM.header.stamp= msg ->header.stamp;
    msg_RPM.header.frame_id = "base_link";
    msg_RPM.rpm_fl = (60* GEAR_RATIO *( (-WHEEL_POSITION_ALONG_X - WHEEL_POSITION_ALONG_Y)* msg->twist.angular.z + msg->twist.linear.x - msg->twist.linear.y) )/ WHEEL_RADIUS;
    msg_RPM.rpm_fr = (60* GEAR_RATIO *( (+WHEEL_POSITION_ALONG_X + WHEEL_POSITION_ALONG_Y)* msg->twist.angular.z + msg->twist.linear.x + msg->twist.linear.y) )/ WHEEL_RADIUS;
    msg_RPM.rpm_rr = (60* GEAR_RATIO *( (+WHEEL_POSITION_ALONG_X + WHEEL_POSITION_ALONG_Y)* msg->twist.angular.z + msg->twist.linear.x - msg->twist.linear.y) )/ WHEEL_RADIUS;
    msg_RPM.rpm_rl = (60* GEAR_RATIO *( (-WHEEL_POSITION_ALONG_X - WHEEL_POSITION_ALONG_Y)* msg->twist.angular.z + msg->twist.linear.x + msg->twist.linear.y) )/ WHEEL_RADIUS;

    pub_rpm.publish(msg_RPM);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "pub_speedcontroller");
	ros::NodeHandle n;
	ros::Subscriber sub;

   	n.getParam("robotDimensionL", WHEEL_POSITION_ALONG_X); 
   	n.getParam("robotDimensionW", WHEEL_POSITION_ALONG_Y); 
   	n.getParam("robotDimensionR", WHEEL_RADIUS); 
    	n.getParam("gearRatio", GEAR_RATIO);
    
    	pub_rpm = n.advertise<robo1::WheelsRPM>("/wheels_rpm", 1000);
    	sub = n.subscribe("/cmd_vel", 1000, calculate_RPM);
    
    ros::spin();

    return 0;
}
