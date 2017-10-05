#ifndef __IMAGE_H_INCLUDED__
#define __IMAGE_H_INCLUDED__
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "control.h"

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window limit";

typedef struct Figure{
    double width;
    double height;
    double r;
}figure;


class ImageConverter
{
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;

    public:
        figure mark;
        figure ref;
        ImageConverter();
        ~ImageConverter();
        void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif
