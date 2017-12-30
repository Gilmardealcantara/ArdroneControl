#ifndef __CONTROL_H_INCLUDED__
#define __CONTROL_H_INCLUDED__

#include <ros/ros.h>
//#include "tum_ardrone/filter_state.h"
//#include "../../../../devel/include/tum_ardrone/filter_state.h"
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>


class Control
{
	private:
		ros::NodeHandle nh_;
		ros::Publisher  cmd_vel_pub_,
						take_off_pub_,
						land_pub_;

		ros::Subscriber nav_data_sub_;
		ros::Subscriber predicted_pose_sub_;
		
		std_msgs::Empty msgEmpty;
        geometry_msgs::Twist base_cmd;
	
	public:
        Control();	
		~Control();
        void navDataReceive(const ardrone_autonomy::Navdata& msg);
		//void droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif
