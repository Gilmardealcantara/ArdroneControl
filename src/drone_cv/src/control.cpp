#include "control.h"

Control::Control()
{
	//predicted_pose_sub_ = nh_.subscribe("/ardrone/predictedPose", 1,
	//		&Control::droneposeCb, this);
	nav_data_sub_ = nh_.subscribe("/ardrone/navdata", 1,
			&Control::navDataReceive, this);

	cmd_vel_pub_	= nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	take_off_pub_	= nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);         
	land_pub_		= nh_.advertise<std_msgs::Empty>("/ardrone/land", 1, true);         
}

	
Control::~Control()
{
}

void Control::navDataReceive(const ardrone_autonomy::Navdata& msg)
{
	printf("get data..\n");
	//ROS_INFO("get data!");
}

/*
void Control::droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr)
{
	ROS_INFO_STREAM("get data!");
}
*/
