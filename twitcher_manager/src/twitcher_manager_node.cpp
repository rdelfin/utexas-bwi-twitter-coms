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

#include "twitcher_connection/Tweet.h"
#include "twitcher_interpreter/dialog_message.h"

ros::Publisher dialogMessagePublisher;

void tweetReceived(const twitcher_connection::Tweet::ConstPtr&);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_manager_node");
    
    ros::NodeHandle node;
    
    dialogMessagePublisher = node.advertise<twitcher_interpreter::dialog_message>("dialog", 1000);
    
    ros::Subscriber subscriber = node.subscribe("twitter_mentions", 1000, 
                                                tweetReceived);
    
    ros::spin();
}


void tweetReceived(const twitcher_connection::Tweet::ConstPtr& tweet)
{
    twitcher_interpreter::dialog_message msg;
    msg.message = tweet->message;
    msg.user_id = tweet->sender;
    msg.datetime = tweet->sentTime;
    
    dialogMessagePublisher.publish(msg);
}
