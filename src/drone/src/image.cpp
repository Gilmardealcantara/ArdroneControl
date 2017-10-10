#include "../include/drone/image.h"

ImageConverter::ImageConverter(Control *c): it_(nh_)
{
	//  Subscrive to input video feed and publish output video feed
	image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
			&ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/output_video", 1);

	cv::namedWindow(OPENCV_WINDOW);
	//cv::namedWindow(OPENCV_WINDOW2);
	control = c;
	control->init();
}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(OPENCV_WINDOW);
	//cv::destroyWindow(OPENCV_WINDOW2);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		cv::Mat orig_image = cv_ptr->image.clone();

		cv::medianBlur(cv_ptr->image, cv_ptr->image, 3);

		// convet to HSV
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
		control->run(mark, ref, found);

		cv::circle(orig_image, cv::Point(mark.width, mark.height), mark.r, CV_RGB(0,255,0), 5);
		cv::circle(orig_image, cv::Point(ref.width, ref.height), 10, CV_RGB(0,0,255), 7);
		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, orig_image);
		//cv::imshow(OPENCV_WINDOW2, red_hue_image);
		cv::waitKey(3);
	
		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());	
	}catch(cv::Exception& e){
			ROS_ERROR("CV exception: %s", e.what());
	}catch(int e){
			ROS_ERROR("Exception: %d", e);
	}
}
