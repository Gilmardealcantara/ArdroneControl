#ifndef __IMAGE_H_INCLUDED__
#define __IMAGE_H_INCLUDED__
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "utils.h"
#include "control.h"

class ImageConverter
{
	private:
		ros::NodeHandle nh_;
		Control *control = new Control(400, true);
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		bool found = false;
    
	public:
        figure mark;
        figure ref;
        
		ImageConverter();
        ~ImageConverter();
        void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif
