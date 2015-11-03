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

#include <actionlib/client/simple_action_client.h>

#include "twitcher_connection/SendTweetAction.h"

int main(int argc, char* argv[])
{
    if(argc != 2) {
        ROS_INFO("This program only accepts one argument!");
        exit(-1);
    }
    
    ros::init(argc, argv, "twitcher_manager_node");
    
    actionlib::SimpleActionClient<twitcher_connection::SendTweetAction> ac("send_tweet", true);
    
    twitcher_connection::SendTweetGoal goal;
    goal.message = argv[1];
    goal.account = "";
    
    
    ac.waitForServer();
    ac.sendGoal(goal);
    
    ac.waitForResult(ros::Duration(5));
    
    ROS_INFO("Tweet sent!");
    
    ros::spin();
}
