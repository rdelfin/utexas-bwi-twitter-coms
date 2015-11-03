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

#include <ros/ros.h>
#include <string>

#include "twitcher_connection/TwitterRequestHandler.h"
#include "twitcher_connection/TwitterApiCall.h"
#include "twitcher_connection/TwitterMentions.h"
#include "twitcher_connection/TwitterUpdateStatus.h"

#include <twitcher_connection/SendTweetAction.h>

#include <actionlib/client/simple_action_client.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_connection_node");
    
    actionlib::SimpleActionClient<twitcher_connection::SendTweetAction> ac("SendTweet", true);
    
    ROS_INFO("Waiting for server");
    ac.waitForServer();
    
    std::string tweet = "YATT: Yet another test tweet. There should be a twitter "
                        "markup language. #apitest";
    twitcher_connection::SendTweetGoal goal;
    goal.account = "rickyd200";
    goal.message = tweet;
    
    ROS_INFO("Sending tweet: \"%s\"", tweet.c_str());
    
    ac.sendGoal(goal);
    
    bool finished = ac.waitForResult(ros::Duration(30.0));
    
    if(finished)
    {
        ROS_INFO("Tweet sent successfully!");
    }
    else
        ROS_INFO("Tweet failed to be sent in the alloted time slot!");

    return 0;
    
}

