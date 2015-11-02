#include <ros/ros.h>
#include <string>

#include "twitcher_connection/TwitterRequestHandler.h"
#include "twitcher_connection/TwitterApiCall.h"
#include "twitcher_connection/TwitterMentions.h"
#include "twitcher_connection/TwitterUpdateStatus.h"

#include <actionlib/server/simple_action_server.h>
#include "twitcher_connection/SendTweetAction.h"

class SendTweetAction
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<twitcher_connection::SendTweetAction> as_;
	std::string action_name_;

	twitcher_connection::SendTweetFeedback feedback_;
	twitcher_connection::SendTweetResult result_;

public:
	SendTweetAction(std::string name) :
    as_(nh_, name, boost::bind(&SendTweetAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~SendTweetAction(void)
  {
  }

  //SENDS THE TWEET
  void executeCB(const twitcher_connection::SendTweetGoalConstPtr &goal)
  {
  	ros::Rate r(1);
  	bool success = true;

  	feedback_.progress=0;

  	ROS_INFO("Sending tweet");
	
	feedback_.progress+=10;

	TwitterRequestHandler handler;

	TwitterApiCall* api = 
		new TwitterUpdateStatus("~/Documents/twitter_config.json", 
	                                goal->message,
	                                -1, false, false);

	std::string resultString = handler.makeRequest(api);

	result_.success = success;

	ROS_INFO("Tweet sent");
	as_.setSucceeded(result_);

  }


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SendTweet");

	SendTweetAction sendTweet(ros::this_node::getName());
	ros::spin();

	return 0;
}