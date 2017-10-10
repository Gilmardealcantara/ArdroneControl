#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>

#include "../include/drone/image.h"
#include "../include/drone/control.h"

int flag = 0;
Control *control = nullptr;
ImageConverter *ic = nullptr;

void  ctrlc_handle(int s){
	if(control){
		printf("finishing\n");
		control->altitude = 0;
		control->end();
		ros::shutdown();
		delete control;
		delete ic;
		printf("finished\n");
		flag = 1;
	}else if (flag){
		exit(1);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drone_node", ros::init_options::NoSigintHandler);
	ros::start();	
	// ctrlc
	struct sigaction sigIntHandler;
   	sigIntHandler.sa_handler = ctrlc_handle;
   	sigemptyset(&sigIntHandler.sa_mask);
   	sigIntHandler.sa_flags = 0;
	
   	sigaction(SIGINT, &sigIntHandler, NULL);

	ros::Rate loop_rate(100);
	// init and end
    //system("rosservice call /ardrone/togglecam"); //simu
    //system("rosservice call /ardrone/setcamchannel \"channel: 1\"");
	control = new Control(1000, true);
    ic = new ImageConverter(control);
	
	//control->init();
	/*
	while(ros::ok()){
		loop_rate.sleep();
		control->stabilize();	
		ros::spinOnce();
	}
	*/
	ros::spin();
	delete ic;	
    return 0;
}
