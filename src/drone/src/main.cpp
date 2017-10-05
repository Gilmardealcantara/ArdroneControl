#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>

#include "../include/drone/image.h"
#include "../include/drone/control.h"

int flag = 0;
Control *control = NULL;

void  ctrlc_handle(int s){
	//control = new Control(1000, true);
	if(control){
		printf("finishing\n");
		control->altitude = 0;
		control->end();
		ros::shutdown();
		delete control;
		printf("finished\n");
		flag = 1;
	}else if (flag){
		exit(1);
	}
}

int main(int argc, char** argv)
{
    
	ros::init(argc, argv, "drone_node", ros::init_options::NoSigintHandler);
	
	// ctrlc
	struct sigaction sigIntHandler;
   	sigIntHandler.sa_handler = ctrlc_handle;
   	sigemptyset(&sigIntHandler.sa_mask);
   	sigIntHandler.sa_flags = 0;
	
   	sigaction(SIGINT, &sigIntHandler, NULL);

		//ros::Rate loop_rate(10);
	// init and end
    //system("rosservice call /ardrone/togglecam"); //simu
    //system("rosservice call /ardrone/setcamchannel \"channel: 1\"");
    //ImageConverter ic;
	control = new Control(1000, true);
	control->init();

	ros::spin();
		
    return 0;
}
