#include <ros/ros.h>
#include <string>



#include <actionlib/server/simple_action_server.h>
#include "twitcher_connection/SendTweetAction.h"

class SendTweetServer
{
public:
    SendTweetServer(std::string name);
  
  void executeCB(const twitcher_connection::SendTweetGoalConstPtr &goal);

  ~SendTweetServer(void) { }
  
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<twitcher_connection::SendTweetAction> as_;
    std::string action_name_;

    twitcher_connection::SendTweetFeedback feedback_;
    twitcher_connection::SendTweetResult result_;
};