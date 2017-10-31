#include "../include/drone/control.h"

Control::Control(int set_altd, bool set_init)
{
	ALTD = set_altd;
	INIT = set_init;
		
	base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	nav_data_sub_ = nh_.subscribe("/ardrone/navdata", 1,
			&Control::navDataReceive, this);
	
	take_off_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);         
	land_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1, true);         
}

	
Control::~Control()
{
}

void Control::init(){
	if(INIT){
		take_off_pub_.publish(msgEmpty);
	}
}

double Control::pid(double Kp, double Ki, double Kd, 
					double err, double err_ant, double T)
{
	P = Kp*err;
	// vei do ceu .... sa pora ta errada animal
	I += Ki*err*T;
	D = (Kd*(err - err_ant))/T;
	//printf("P: %lf, I: %lf, D: %lf\n", P, I, D);
	return (P + I + D);
}

void Control::end(){
	if(INIT){
		printf("Landing of %d mm\n", altitude);
		while(altitude > 200){
			printf("Landing of %d mm\n", altitude);
		   	base_cmd.linear.x = 0; 
		   	base_cmd.linear.y = 0;
		   	base_cmd.linear.z = -0.7;
		   	base_cmd.angular.z = 0;
		   	base_cmd.angular.x = 0;
		   	base_cmd.angular.y = 0;
			ros::spinOnce();
		}
		land_pub_.publish(msgEmpty);
		land_pub_.publish(msgEmpty);
		land_pub_.publish(msgEmpty);
		printf("Land ....\n");
    }
}
/* control nano seconds
mean:  6.72329e+06
max:   5.86789e+07
*/
void Control::navDataReceive(const ardrone_autonomy::Navdata& msg){
	current_msg = msg; 
	stabilize();
	showData();	
}

//mean:  8.41395e+06 *10e-9 =~	8.4e-4	= 0.00084
//max:   5.33942e+07 *10e-9 =~	5.3e-3	= 0.00534
void Control::stabilize(){
	double	err_lz = (double)(ALTD - current_msg.altd), 
			err_az = - (double)(current_msg.rotZ)/(180); // -1 : 1
	//printf("err: %lf %lf\n", err_lz, err_az);
	double vel_lz =  pid(0.001, 0, 0, err_lz, 0 , 1);
	double vel_az = pid(1, 0, 0, err_az, 0 , 1);
	//printf("vel: %lf %lf\n", vel_lz, vel_az);
	
	//base_cmd.linear.x = 0.0;//- (double)(current_msg.vx)/1000; 
	//base_cmd.linear.y = 0.0;//- (double)(current_msg.vy)/1000;
	base_cmd.linear.z = 0.0;//vel_lz;
	
	base_cmd.angular.z = vel_az;
	altitude = current_msg.altd;	
	//cmd_vel_pub_.publish(base_cmd);
}

void Control::showData(){
	printf("\n\n\n*****Data******\n");
	printf("\n*Receive: \nvx: %f\nvy: %f\nvz: %f\n"	
					   "\nrotX: %f\nrotY: %f\nrotZ: %f\nalt: %d\n", 
											current_msg.vx, 
											current_msg.vy, 
											current_msg.vz,
											current_msg.rotX,
											current_msg.rotY,
											current_msg.rotZ,
											current_msg.altd
											);

	printf("\n**Send:\nvlx: %f\nvly: %f\nvlz: %f\n"
				  "\nvax: %f\nvay: %f\nvaz: %f\n",
											base_cmd.linear.x,
											base_cmd.linear.y,
											base_cmd.linear.z, 
											base_cmd.angular.x, 
											base_cmd.angular.y, 
											base_cmd.angular.z
											);	
	
	printf("\n***Control Actions: \nP: %lf\nI: %lf\nD: %lf\n", P, I, D);
}

void Control::set_cmd(double xl, double yl)
{
	base_cmd.linear.x = xl; 
	base_cmd.linear.y = yl;

	cmd_vel_pub_.publish(base_cmd);
}

//mean:  2.68742e+07 * 10e-9 =~  2.6e-3 = 0.00269
//max:   5.31511e+07 * 10e-9 =~  5.3e-3 = 0.00532

void Control::run(Figure mark, Figure ref, bool found)
{
	double real_errx = mark.height - ref.height; 
	double real_erry = mark.width - ref.width; 
	errx = found ? - (double)real_errx/(180.00): 0.00, //180
	erry = found ? -(double)real_erry/(360.00) : 0.00; //360
	
	double	kp = 0.5, 
			ki = 0.5, 
			kd = 0.001,
			T  = 0.005; // 200Hz
	double linear_x = pid(kp, ki, kd, errx, errx_ant, T),
		   linear_y = pid(kp, ki, kd, erry, errx_ant, T);
	
	errx_ant = errx;
	erry_ant = erry;
	
	set_cmd(linear_x, linear_y); 
	//set_cmd(0, 0); 
}

