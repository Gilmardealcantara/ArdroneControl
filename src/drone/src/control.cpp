#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

void MagCallback(const geometry_msgs::Vector3Stamped::ConstPtr&  msg){
    double x = msg->vector.x;
    double y = msg->vector.y;
    double z = msg->vector.z;
    ROS_INFO("x: %.3f, y: %.3f, z: %.3f", x, y, z);
    //ROS_WARN("x: %.3f, y: %.3f, z: %.3f", x, y, z);
    //ROS_DEBUG("x: %.3f, y: %.3f, z: %.3f", x, y, z);
    //ROS_FATAL("x: %.3f, y: %.3f, z: %.3f", x, y, z);
    //ROS_ERROR("x: %.3f, y: %.3f, z: %.3f", x, y, z);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "location_monitor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("magnetic", 10, MagCallback);
    ros::spin();

    return 0;
}
