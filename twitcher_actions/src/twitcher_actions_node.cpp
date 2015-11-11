#include <ros/ros.h>

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_actions");
    
    ros::NodeHandle n;
    
    ros::spin();
}