#include "ros/ros.h"   
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "robo1/MessageOdometry.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include "robo1/CalibrationMessage.h"      
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <robo1/fix_parametersConfig.h>

double prev_ticks0, prev_ticks1 , prev_ticks2, prev_ticks3;
double l, r , w, gear_rateo,  cpr;
ros::Time prev_ts;
ros::Publisher pub_Calibration;

/*void fix_parameters(robo1::fix_parametersConfig &config, uint32_t level){    //DYNAMIC RECONFIGURE TO CALIBRATE THE PARAMETERS MANUALLY
                                                                             											  
	l=config.L;
	r=config.R;
	w=config.W;
	cpr=config.CPR;  
}*/

void updateRoboValues(const sensor_msgs::JointState::ConstPtr& msg){

	robo1::CalibrationMessage cal_msg;	

	/*double u0= (msg -> velocity[0])/ (gear_rateo* 60);    //msg1 -> velocity[0] == radiant/min  ALGORITHM WITH RPM 
	double u1= (msg -> velocity[1])/ (gear_rateo* 60); 
	double u2= (msg -> velocity[2])/ (gear_rateo* 60); 
	double u3= (msg -> velocity[3])/ (gear_rateo* 60);*/

	double rpm0= (msg -> position[0] - prev_ticks0) /( (msg -> header.stamp - prev_ts).toSec()* cpr);   //rpm0 == round/sec   ALGORITHM WITH TICKS
	double rpm1= (msg -> position[1] - prev_ticks1) /( (msg -> header.stamp - prev_ts).toSec()* cpr);
	double rpm2= (msg -> position[2] - prev_ticks2) /( (msg -> header.stamp - prev_ts).toSec()* cpr);
	double rpm3= (msg -> position[3] - prev_ticks3) /( (msg -> header.stamp - prev_ts).toSec()* cpr);

	double u0= (rpm0* 2* 3.14)/ (gear_rateo); 
	double u1= (rpm1* 2* 3.14)/ (gear_rateo); 
	double u2= (rpm2* 2* 3.14)/ (gear_rateo); 
	double u3= (rpm3* 2* 3.14)/ (gear_rateo);

	prev_ts= msg -> header.stamp;
	prev_ticks0= msg -> position[0];	
	prev_ticks1= msg -> position[1];
	prev_ticks2= msg -> position[2];
	prev_ticks3= msg -> position[3];

	cal_msg.header.stamp = msg -> header.stamp;
    	cal_msg.header.frame_id = "base_link";
    	cal_msg.u0 = u0;
    	cal_msg.u1 = u1;
    	cal_msg.u2 = u2;
    	cal_msg.u3 = u3;
    	/*cal_msg.W = w; 
    	cal_msg.L = l;
	cal_msg.R = r;
    	cal_msg.CPR = cpr;
    	cal_msg.GEAR_RATEO = gear_rateo;*/
    	pub_Calibration.publish(cal_msg);
    
}


int main(int argc, char **argv){
   
    ros::init(argc, argv, "pub_calibration");
    ros::NodeHandle n;
    ros::Subscriber sub;

    /*n.getParam("robotDimensionL", l); 
    n.getParam("robotDimensionW", w); 
    n.getParam("robotDimensionR", r); */
    n.getParam("gearRatio", gear_rateo);
    n.getParam("cpr", cpr);

    /*dynamic_reconfigure::Server<robo1::fix_parametersConfig> dynServer;
    dynamic_reconfigure::Server<robo1::fix_parametersConfig>::CallbackType f;
    f = boost::bind(&fix_parameters,_1, _2);
    dynServer.setCallback(f);*/

    pub_Calibration = n.advertise<robo1::CalibrationMessage>("/calibration_msg", 1000);    
    sub = n.subscribe("/wheel_states", 1000, updateRoboValues);

    ros::spin();
    return 0;
}
