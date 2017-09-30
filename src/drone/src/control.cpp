#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <cmath>        // std::abs
#include <signal.h>

#define ALTD 1000
#define INIT true

void ctrlc_handle(int s);
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window limit";

typedef struct Figure{
    double width;
    double height;
    double r;
}figure;

class Control
{ 
    ros::NodeHandle nh_;
    ros::Publisher  cmd_vel_pub_,
                    take_off_pub_,
                    land_pub_;
    ros::Subscriber nav_data_sub_;
    
    std_msgs::Empty msgEmpty;
    struct sigaction sigIntHandler;

    public:
        int altitude;
        geometry_msgs::Twist base_cmd;
    
        Control()
        {
            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
            nav_data_sub_ = nh_.subscribe("/ardrone/navdata", 1,
                    &Control::navDataReceive, this);
           
            
            if(INIT){
                // init and end
                signal(SIGINT, ctrlc_handle);
                //take of 
                take_off_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);         
                land_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1, true);         
                take_off_pub_.publish(msgEmpty);
                //zeroAllVell();
                base_cmd.linear.x = 0; 
                base_cmd.linear.y = 0;
                base_cmd.linear.z = 0;
                base_cmd.angular.z = 0;
                base_cmd.angular.x = 0;
                base_cmd.angular.y = 0;
            }
        }
        
        ~Control()
        {
            land_pub_.publish(msgEmpty);
        
        }

        void land(){
            land_pub_.publish(msgEmpty);
        } 
        
        void zeroAllVel(){
            base_cmd.linear.x = 0; 
            base_cmd.linear.y = 0;
            //base_cmd.linear.z = 0;
            base_cmd.angular.z = 0;
            base_cmd.angular.x = 0;
            base_cmd.angular.y = 0;
        }

        void navDataReceive(const ardrone_autonomy::Navdata& msg){
            //zeroAllVel(); 
            altitude = msg.altd;
            double vel_lz = (double)(ALTD - msg.altd)/1000;
            double vel_az = - (double)(msg.rotZ)/180;
            //printf("\nrotZ: %.2f, velaz: %.2f\n", msg.rotZ, vel_az);
            //printf("\nvelX: %f, velY: %f,  velZ: %f\n", msg.vx, msg.vy, msg.vz);
            base_cmd.angular.z = vel_az;
            base_cmd.linear.z = vel_lz;
            //cmd_vel_pub_.publish(base_cmd);
        }

        void set_cmd(double xl, double yl)
        {
            base_cmd.linear.x = xl; 
            base_cmd.linear.y = yl;

            cmd_vel_pub_.publish(base_cmd);
        }
         
        void run(Figure mark, Figure ref, bool found)
        {
            double errx = mark.height - ref.height; 
            double erry = mark.width - ref.width; 

            //bool cond = (found && (altitude > (ALTD - 5) && altitude < ALTD)
            //            && (base_cmd.angular.z > -5 ) && (base_cmd.angular.z < 5));

            // double linear_x = found ? - (double)errx/11520.00: 0.00, //180
            //        linear_y = found ? -(double)erry/20480.00 : 0.00; //360
            int mul = 1;
            double linear_x = found ? - (double)errx/(180.00*mul): 0.00, //180
                   linear_y = found ? -(double)erry/(360.00*mul) : 0.00; //360
            
            double  vel = 0.5, raio = 1;
            
            printf("\n\n\n\nRef(%.2f, %.2f), Alvo(%.2f, %.2f),"
                    "\nraio:  %.2f, alt    %d\n"
                    "\nerrx:  %.2f, erry:  %.2f"
                    "\nvellx: %.4f, velly: %.4f"
                    "\nvelaz: %.2f, vellz: %.2f\n", 
                    ref.width, ref.height, mark.width, mark.height, mark.r, altitude,
                    errx, erry, linear_x, linear_y, 
                    base_cmd.angular.z, base_cmd.linear.z);
            set_cmd(linear_x, linear_y); 
            //set_cmd(0, 0); 
        }
};
void ctrlc_handle(int s){
    Control c;
    while(c.altitude > 200){
           c.base_cmd.linear.x = 0; 
           c.base_cmd.linear.y = 0;
           c.base_cmd.linear.z = -0.7;
           c.base_cmd.angular.z = 0;
           c.base_cmd.angular.x = 0;
           c.base_cmd.angular.y = 0;
    }
    c.land();
    printf("***STOP*** %d\n",s);
    exit(1);
}

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    Control control;

    public:
        figure mark;
        figure ref;
        
        ImageConverter(): it_(nh_)
        {
            //  Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
                    &ImageConverter::imageCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);

            cv::namedWindow(OPENCV_WINDOW);
            //cv::namedWindow(OPENCV_WINDOW2);
        }

        ~ImageConverter()
        {
            cv::destroyWindow(OPENCV_WINDOW);
            //cv::destroyWindow(OPENCV_WINDOW2);
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat orig_image = cv_ptr->image.clone();

            cv::medianBlur(cv_ptr->image, cv_ptr->image, 3);

            // convet tho HSV
            cv::Mat hsv_image;
            cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

            // Threshold the HSV image, keep only the red pixels
            cv::Mat lower_red_hue_range;
            cv::Mat upper_red_hue_range;
            cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

            // Combine the above two images
            cv::Mat red_hue_image;
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

            cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

            // Use the Hough transform to detect circles in the combined threshold image
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

            // Loop over all detected circles and outline them on the original image
            ref.width = cv_ptr->image.size().width/2;
            ref.height = cv_ptr->image.size().height/2;
            double raio = 0;
            bool found = false;
            for(int i = 0; i < circles.size(); i++){
                //cv::circle(cv_ptr->image, cv::Point(circles[0][0], circles[0][1]), circles[0][2], CV_RGB(0,255,0), 5);
                if(circles[i][2] > raio){
                    mark.width = circles[i][0];
                    mark.height = circles[i][1];
                    mark.r = raio = circles[i][2];
                    found = true;
                }
                //printf("Referencia(%f, %f), Alvo(%f, %f), raio: %f\n", ref.x, ref.y, mark.x, mark.y, mark.r);
            }
            control.run(mark, ref, found);

            cv::circle(orig_image, cv::Point(mark.width, mark.height), mark.r, CV_RGB(0,255,0), 5);
            cv::circle(orig_image, cv::Point(ref.width, ref.height), 10, CV_RGB(0,0,255), 7);
            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, orig_image);
            //cv::imshow(OPENCV_WINDOW2, red_hue_image);
            cv::waitKey(3);

            // Output modified video stream
            //image_pub_.publish(cv_ptr->toImageMsg());
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    //system("rosservice call /ardrone/togglecam"); //simu
    //system("rosservice call /ardrone/setcamchannel \"channel: 1\"");
    ImageConverter ic;
    //Control co;
    ros::spin();
    return 0;
}
