#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_connection_node");
    
    ROS_INFO("Hello world!");
    
    while(ros::ok()) {
        ros::spinOnce();
    }
}