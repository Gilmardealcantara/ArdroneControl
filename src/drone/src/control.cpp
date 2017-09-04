#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <cmath>        // std::abs

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window limit";

typedef struct Figure{
    double x;
    double y;
    double r;
}figure;

class Control
{ 
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    public:
        geometry_msgs::Twist base_cmd;
    
        Control()
        {
            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
        }

        void set_cmd(double xl, double yl, double zl, double za)
        {
            base_cmd.linear.x = xl; 
            base_cmd.linear.y = yl;
            base_cmd.linear.z = zl;
            base_cmd.angular.z = za; 
            cmd_vel_pub_.publish(base_cmd);
        }
        
        void run(Figure mark, Figure ref)
        {
            double errx = mark.x - ref.x; 
            double erry = mark.y - ref.y; 
            printf("Ref(%.2f, %.2f), Alvo(%.2f, %.2f), r: %.2f\t errx:%.2f, erry:%.2f\n", 
                    ref.x, ref.y, mark.x, mark.y, mark.r,
                    errx, erry);

            double linear_x = 0, 
                   linear_y = 0, 
                   vel = 1.0, raio = 20.0;
            /*if(errx > raio)
                linear_x = -vel;
            else if (errx < -raio)
                linear_x = vel;

            if(erry > raio)
                linear_y = -vel;
            else if(erry < -raio)
                linear_y = vel;
            */
            set_cmd(linear_x, linear_y, 0.0, 0.0); 
        }
};


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
            // Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe("ardrone/image_raw", 1,
                    &ImageConverter::imageCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);

            cv::namedWindow(OPENCV_WINDOW);
            cv::namedWindow(OPENCV_WINDOW2);
        }

        ~ImageConverter()
        {
            cv::destroyWindow(OPENCV_WINDOW);
            cv::destroyWindow(OPENCV_WINDOW2);
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
            ref.x = cv_ptr->image.size().width/2;
            ref.y = cv_ptr->image.size().height/2;
            if(circles.size() == 1){
                //cv::circle(cv_ptr->image, cv::Point(circles[0][0], circles[0][1]), circles[0][2], CV_RGB(0,255,0), 5);
                mark.x = circles[0][0];
                mark.y = circles[0][1];
                mark.r = circles[0][2];
                control.run(mark, ref);
                //printf("Referencia(%f, %f), Alvo(%f, %f), raio: %f\n", ref.x, ref.y, mark.x, mark.y, mark.r);
            }
            cv::circle(cv_ptr->image, cv::Point(mark.x, mark.y), mark.r, CV_RGB(0,255,0), 5);
            cv::circle(cv_ptr->image, cv::Point(ref.x, ref.y), 10, CV_RGB(0,0,255), 7);
            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::imshow(OPENCV_WINDOW2, red_hue_image);
            cv::waitKey(3);

            // Output modified video stream
            //image_pub_.publish(cv_ptr->toImageMsg());
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    system("rosservice call /ardrone/setcamchannel \"channel: 1\"");
    ImageConverter ic;
    ros::spin();
    return 0;
}
