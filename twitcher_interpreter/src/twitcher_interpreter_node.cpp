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

#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "json/json.hpp"

#include <twitcher_actions/GoToLocationAction.h>

#include "twitcher_interpreter/dialog_message.h"
#include "twitcher_interpreter/location.h"

#include <std_msgs/Float32.h>

using json = nlohmann::json;

boost::regex goToTweetRegex;
actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>* client;

std::vector<Location> loc;


void messageReceiver(const twitcher_interpreter::dialog_message::ConstPtr&);
void initLocations();

int main(int argc, char* argv[])
{
    initLocations();
    
    goToTweetRegex = boost::regex("(?<=Go to room )l3_([\\d]{3}|414[ab])",
                                  boost::regex::icase);
    
    ros::init(argc, argv, "twitcher_interpreter_node");
    ros::NodeHandle node;
    
    client = new actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>(node, "GoToLocation", true);
    
    client->waitForServer();
    
    ROS_INFO_STREAM("Twitcher Interpreter Node up, listening on /GoToLocation");
    
    ros::Subscriber subscriber = node.subscribe("dialog", 1000, messageReceiver);
    
    ros::spin();
    
    delete client;
}

void initLocations() {
    std::ifstream file_stream("/home/rdelfin/Documents/rooms.json");
    json root;
    
    file_stream >> root;
    
    for(int i = 0; i < root.size(); i++) {
        loc.push_back(Location(root[i].dump()));
    }
}

void messageReceiver(const twitcher_interpreter::dialog_message::ConstPtr& msg)
{
    ROS_INFO_STREAM("Interpreter received message: \"" << msg->message << 
                    "\" with timestamp " << msg->datetime);
    
    
    
    for(auto it = loc.begin(); it != loc.end(); ++it) {
        if(it->isMentioned(msg->message)) {
            twitcher_actions::GoToLocationGoal goal;
            goal.location_name = it->get_asp_name();
            client->sendGoal(goal);
            break;
        }
    }
}
