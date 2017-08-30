#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>

static const std::string OPENCV_WINDOW = "Image window";

typedef struct Figure{
    double x;
    double y;
    double r;
}figure;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

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
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
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
            printf("Referencia(%f, %f), Alvo(%f, %f), raio: %f\n", ref.x, ref.y, mark.x, mark.y, mark.r);
        }
        cv::circle(cv_ptr->image, cv::Point(mark.x, mark.y), mark.r, CV_RGB(0,255,0), 5);
        cv::circle(cv_ptr->image, cv::Point(ref.x, ref.y), 10, CV_RGB(0,0,255), 7);
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
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
