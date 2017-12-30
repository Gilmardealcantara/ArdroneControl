#include <ros/ros.h>
#include <ros/ros.h>
//#include "tum_ardrone/filter_state.h"
#include "../../../devel/include/tum_ardrone/filter_state.h"
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <ctime>
#include <signal.h>

class Control{
public:
	Control (ros::NodeHandle &nh_)
	{
		predicted_pose_sub_ = nh_.subscribe("/ardrone/predictedPose", 1,
				&Control::droneposeCb, this);
		nav_data_sub_ = nh_.subscribe("/ardrone/navdata", 1,
				&Control::navDataReceive, this);

		cmd_vel_pub_		= nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
		take_off_pub_		= nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);         
		tum_ardrone_pub_	= nh_.advertise<std_msgs::String>("/tum_ardrone/com", 50);
		land_pub_			= nh_.advertise<std_msgs::Empty>("/ardrone/land", 1, true);         
		
		std_msgs::Empty msgEmpty;
		take_off_pub_.publish(msgEmpty);
		float last_st;
	}
	
	~Control()
	{
	}
	void land()
	{
		land_pub_.publish(msgEmpty);
	}
	void navDataReceive(const ardrone_autonomy::Navdata& msg){
		//base_cmd.angular.z = - (double)(msg.rotZ + 45)/(360) ;	
		//ROS_INFO("navdata:  \nvx: %f \tvy: %f \tvz: %f"
		//			    "\nax: %f \tay: %f \taz: %f\n", 
		//					msg.vx, msg.vy, msg.vz, msg.ax, msg.ay, msg.az); 
		//cmd_vel_pub_.publish(base_cmd);
	}


	void droneposeCb2(const tum_ardrone::filter_stateConstPtr st){
		if(lastT = 0) return;
		time_t now = time(0);
		//float dT = now - lastT;
		float dT = 1;
		float x = st->x, y = st->y;
		float raio = 1, vel_max = 0.5;
		float vx =  - ((pow(x, 4) + pow(y, 4) - pow(raio, 4)) * 4 * pow(x, 3))/ pow(raio, 4) - 4 * pow(y, 3);
		float vy =  - ((pow(x, 4) + pow(y, 4) - pow(raio, 4)) * 4 * pow(y, 3))/pow(raio, 4) + 4 * pow(x, 3);
		float x_d = st->x + vx;
		float y_d = st->y + vy;
		

		std::string cmd("c goto " + std::to_string(x_d) + " " + std::to_string(y_d)+ " 0.000000 0.00000");
		//std::string cmd("c goto 0.00 0.00 0.00 0.00");
		std_msgs::String s;
		s.data = cmd.c_str(); 
		tum_ardrone_pub_.publish(s);
		
		std::cout << "pub comand:'" << cmd << "'\tdT:" << dT << std::endl;	
		std::cout << "velx:" << vx << "\tvely:" << vy << std::endl;	
		lastT = now;

	}
	void droneposeCb(const tum_ardrone::filter_stateConstPtr st)
	{
		float raio = 2, vel_max = 0.5;
		float x = st->x, y = st->y;
		
		if(x == 0.0000 || y == 0.0000 ){
			cmd_vel_pub_.publish(base_cmd);
			return;
		};

		//float vx =  - (pow(x, 4) + pow(y, 4) - raio) * 4 * pow(x, 3) - 4 * pow(y, 3);
		//float vy =  - (pow(x, 4) + pow(y, 4) - raio) * 4 * pow(y, 3) + 4 * pow(x, 3);
		float vx =  - ((pow(x, 4) + pow(y, 4) - pow(raio, 4)) * 4 * pow(x, 3))/ pow(raio, 4) - 4 * pow(y, 3);
		float vy =  - ((pow(x, 4) + pow(y, 4) - pow(raio, 4)) * 4 * pow(y, 3))/pow(raio, 4) + 4 * pow(x, 3);
			
		float vx_n = 0.5 * (vx / (sqrt(pow(vx, 2) + pow(vy, 2)) + 0.0001));	
		float vy_n = 0.5 * (vy / (sqrt(pow(vx, 2) + pow(vy, 2)) + 0.0001 ));	
		float vw = ((st->yaw - 45)/180);
		float vz = - (st->z - 1.000);

		ROS_INFO("\n\nPose:  \nx: %f\ny: %f\nz : %f\n"
						"---------------------------------------"
						"\nvx: %f\nvy: %f\nvz: %f\n"	
						"---------------------------------------"
						"\nroll: %f\npitch: %f\nyaw: %f\n"
						"---------------------------------------"
						"\nvx_d: %f\nvy_d: %f\nvz_d: %f\nvw_d: %f\n"	
						"---------------------------------------"
						"\nvx_n: %f\nvy_n: %f\n"	
						"*************************\n",
												st->x, st->y, st->z, 
												st->dx, st->dy, st->dz,
												st->roll, st->pitch, st->yaw,
												vx, vy, vz, vw, 
												vx_n, vy_n);
		
		
	
		
		base_cmd.linear.x = vx_n;
		base_cmd.linear.y = vy_n;
		base_cmd.linear.z = vz;
		base_cmd.angular.z = vw;
		cmd_vel_pub_.publish(base_cmd);
	}

private:	
		ros::Publisher  cmd_vel_pub_,
						take_off_pub_,
						tum_ardrone_pub_,
						land_pub_;
		ros::Subscriber nav_data_sub_;
		ros::Subscriber predicted_pose_sub_;
		
		std_msgs::Empty msgEmpty;
        geometry_msgs::Twist base_cmd;
	 	float lastT = 0;

};

Control *c;
void ctrlcHandler(int s){
	std::cout << "end ... " << std::endl;
	c->land();
	exit(0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "done_cv_node");
	ros::start();
	ros::NodeHandle nh_;
	signal(SIGINT, ctrlcHandler);
	c = new Control(nh_);	
	ros::spin();
	return 0;
}
