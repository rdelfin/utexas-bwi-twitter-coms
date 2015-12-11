/*
 * Copyright 2015 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include <iostream>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <twitcher_actions/GoToLocationAction.h>

#include "twitcher_interpreter/dialog_message.h"

#include <twitcher_connection/SendTweetAction.h>

boost::regex goToTweetRegex;
actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>* client;
actionlib::SimpleActionClient<twitcher_connection::SendTweetAction>* tweet_client;


void messageReceiver(const twitcher_interpreter::dialog_message::ConstPtr&);

int main(int argc, char* argv[])
{
    
    goToTweetRegex = boost::regex("(?<=Go to room )l3_((414[ab])|[\\d]{3})",
                                  boost::regex::icase);
    
    ros::init(argc, argv, "twitcher_interpreter_node");
    ros::NodeHandle node;
    
    client = new actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>(node, "GoToLocation", true);

    tweet_client = new actionlib::SimpleActionClient<twitcher_connection::SendTweetAction>(node, "send_tweet",true);
    client->waitForServer();
    
    tweet_client->waitForServer();
    ROS_INFO_STREAM("Twitcher Interpreter Node up, listening on /GoToLocation");
    
    ros::Subscriber subscriber = node.subscribe("dialog", 1000, messageReceiver);
    
    ros::spin();
    
    delete client;
    delete tweet_client;
}

void messageReceiver(const twitcher_interpreter::dialog_message::ConstPtr& msg)
{
    ROS_INFO_STREAM("Interpreter received message: \"" << msg->message << 
                    "\" with timestamp " << msg->datetime);
    boost::smatch matchResult;
                          
    std::string message = msg->message;
    boost::match_flag_type flags = boost::match_default;

    std::stringstream tweet_stream;
    twitcher_connection::SendTweetGoal tweet_goal;
    
    /* If regex matches for current tweet, then service request */
    if(boost::regex_search(message, matchResult, goToTweetRegex, flags)) {
        std::string dialogMessage = msg->message;
        twitcher_actions::GoToLocationGoal goal;
        goal.location_name = matchResult[0];
        
        tweet_stream << "@" << msg->user_id << " Alright, going to " << matchResult[0];
	tweet_goal.message = tweet_stream.str();
	tweet_client->sendGoal(tweet_goal);
	client->sendGoal(goal);
    }
    else {
        tweet_stream << "@" << msg->user_id << " Sorry, I don't understand."; 
	tweet_goal.message = tweet_stream.str();
	tweet_client->sendGoal(tweet_goal);
    }    
}
