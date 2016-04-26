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
#include <twitcher_interpreter/interpret_dialog.h>

#include <twitcher_actions/FaceDoorAction.h>
#include <twitcher_actions/GoToLocationAction.h>
#include <twitcher_actions/SayAction.h>

ros::ServiceClient interpreterClient;


actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>* goToLocationClient;
actionlib::SimpleActionClient<twitcher_actions::FaceDoorAction>* faceDoorClient;
actionlib::SimpleActionClient<twitcher_actions::SayAction>* sayClient;

void tweetReceived(const twitcher_connection::Tweet::ConstPtr&);
void actOnTweet(const twitcher_connection::Tweet::ConstPtr& tweet, const twitcher_interpreter::interpret_dialog::Response& res);
void goToLocationAndSay(const twitcher_interpreter::named_location& location, const std::string& text);
void goToLocation(const twitcher_interpreter::named_location& location);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_manager_node");
    
    ros::NodeHandle node;
    
    // Connect to interpreter
    interpreterClient = node.serviceClient<twitcher_interpreter::interpret_dialog>("twitter/interpret_message");
    interpreterClient.waitForExistence();
    
    // Connect to twitter_mentions (from twitcher_connection)
    ros::Subscriber subscriber = node.subscribe("twitter_mentions", 1000, 
                                                tweetReceived);
    
    // Set up action clients for twitcher_actions
    goToLocationClient = new actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>(node, "GoToLocation", true);
    faceDoorClient = new actionlib::SimpleActionClient<twitcher_actions::FaceDoorAction>(node, "FaceDoor", true);
    sayClient = new actionlib::SimpleActionClient<twitcher_actions::SayAction>(node, "SayTwitter", true);

    goToLocationClient->waitForServer();
    faceDoorClient->waitForServer();
    sayClient->waitForServer();
    
    ros::spin();
    
    delete goToLocationClient;
    delete faceDoorClient;
    delete sayClient;
}


void tweetReceived(const twitcher_connection::Tweet::ConstPtr& tweet)
{
    ROS_INFO_STREAM("Manager received tweet from /twitter_mentions. Content: \""
                    << tweet->message << "\". Forwarding to /dialog");
    
    twitcher_interpreter::interpret_dialog::Request req;
    twitcher_interpreter::interpret_dialog::Response res;
    
    req.message = tweet->message;
    req.user_id = tweet->sender;
    req.datetime = tweet->sentTime;
    
    interpreterClient.call(req, res);
    
    actOnTweet(tweet, res);
}

void actOnTweet(const twitcher_connection::Tweet::ConstPtr& tweet, const twitcher_interpreter::interpret_dialog::Response& res) {
    switch(res.action) {
        case twitcher_interpreter::interpret_dialog::Response::GO_TO_ACTION:
            goToLocation(res.loc_args[0]);
            break;
        case twitcher_interpreter::interpret_dialog::Response::GO_TO_AND_SAY:
            goToLocationAndSay(res.loc_args[0], res.string_args[0]);
    }
}


void goToLocationAndSay(const twitcher_interpreter::named_location& location, const std::string& text)
{
    goToLocation(location);
    
    ROS_INFO_STREAM("Starting to speak... Saying \"" << text << "\"");
    
    twitcher_actions::SayGoal say_goal;
    say_goal.message = text;
    sayClient->sendGoal(say_goal);
    sayClient->waitForResult();
    
    ROS_INFO("Say finished!");
}

void goToLocation(const twitcher_interpreter::named_location& location)
{
    if(location.doors.size() != 0) {
        ROS_INFO("Sending face door!");
        
        twitcher_actions::FaceDoorGoal face_goal;
        face_goal.door_name = location.doors[0];
        faceDoorClient->sendGoal(face_goal);
        faceDoorClient->waitForResult();
        
        ROS_INFO("Finished face door action!");
    }
    else {
        ROS_INFO("Sending go to goal! Setup...");
        
        twitcher_actions::GoToLocationGoal goto_goal;
        goto_goal.location_name = location.asp_name;
        goToLocationClient->sendGoal(goto_goal);
        goToLocationClient->waitForResult();
        
        ROS_INFO("Finished go to action!");
    }
}