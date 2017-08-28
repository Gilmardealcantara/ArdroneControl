#include "ros/ros.h"
#include "sensor_msgs/Image.h"

void ImageCallback(const sensor_msgs::Image::ConstPtr&  msg){
    uint32_t n_rows = msg->height;
    uint32_t step = msg->step;
    ROS_INFO("h: %d, w: %d", n_rows, w);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Image_monitor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("ardrone/bottom/image_raw", 10, ImageCallback);
    ros::spin();

    return 0;
}
