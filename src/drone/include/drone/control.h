#ifndef __CONTROL_H_INCLUDED__
#define __CONTROL_H_INCLUDED__

#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#include "utils.h"

class Control
{
	private:
		int ALTD;
		bool INIT;
		ros::NodeHandle nh_;
		ros::Publisher  cmd_vel_pub_,
						take_off_pub_,
						land_pub_;
		ros::Subscriber nav_data_sub_;
		std_msgs::Empty msgEmpty;
			

	public:
        int altitude;
        geometry_msgs::Twist base_cmd;
		ardrone_autonomy::Navdata current_msg;
		 

        Control(int set_alt, bool set_init);	
		~Control();
		void init(); 
        void navDataReceive(const ardrone_autonomy::Navdata& msg);
		void stabilize();
		void showData();
        void set_cmd(double xl, double yl); 
        void run(figure mark, figure ref, bool found);
		void end();
};

#endif
