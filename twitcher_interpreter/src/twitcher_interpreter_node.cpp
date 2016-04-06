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
#include <regex>

#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "json/json.hpp"

#include <twitcher_actions/GoToLocationAction.h>
#include <twitcher_actions/FaceDoorAction.h>
#include <twitcher_actions/SayAction.h>

#include "twitcher_interpreter/dialog_message.h"
#include "twitcher_interpreter/location.h"

using json = nlohmann::json;

actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>* goToLocationClient;
actionlib::SimpleActionClient<twitcher_actions::FaceDoorAction>* faceDoorClient;
actionlib::SimpleActionClient<twitcher_actions::SayAction>* sayClient;

std::vector<Location> loc;
std::regex goToTweetRegex;


void messageReceiver(const twitcher_interpreter::dialog_message::ConstPtr&);
void initLocations();
bool locExists(std::string msg, Location** loc);
void gotoDoorAndSay(Location* location, std::string spoken_text);

int main(int argc, char* argv[])
{
    goToTweetRegex = std::regex("(Go to room )([.\\w\\s]+) and say (.+)",
                                std::regex_constants::grep | std::regex_constants::icase);
    
    initLocations();
    
    ros::init(argc, argv, "twitcher_interpreter_node");
    ros::NodeHandle node;
    
    goToLocationClient = new actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>(node, "GoToLocation", true);
    faceDoorClient = new actionlib::SimpleActionClient<twitcher_actions::FaceDoorAction>(node, "FaceDoor", true);
    
    goToLocationClient->waitForServer();
    
    ROS_INFO_STREAM("Twitcher Interpreter Node up, listening on /GoToLocation");
    
    ros::Subscriber subscriber = node.subscribe("dialog", 100, messageReceiver);
    
    ros::spin();
    
    delete goToLocationClient;
}

void initLocations() {
    std::ifstream file_stream("/home/users/fri/Documents/rooms.json");
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
    
    std::string msgString = msg->message;
    
    auto it = std::sregex_iterator(msgString.begin(), msgString.end(), goToTweetRegex);
    
    // The string was found. Execute action
    if(it != std::sregex_iterator()) {
        std::smatch match = *it;
        std::string location_name = match.str(2);
        std::string spoken_text = match.str(3);
        Location *location;
        if(locExists(location_name, &location)) {
            gotoDoorAndSay(location, spoken_text);
        }
    }
    
    /*
    if(it->hasDoor()) {
        twitcher_actions::FaceDoorGoal goal;
        goal.door_name = it->getFirstDoor();
        faceDoorClient->sendGoal(goal);
    }
    else {
        twitcher_actions::GoToLocationGoal goal;
        goal.location_name = it->getAspName();
        goToLocationClient->sendGoal(goal);
    }
    
    break;
     */
}

bool locExists(std::string msg, Location** result) {
    for(auto it = loc.begin(); it != loc.end(); ++it) {
        if(it->isMentioned(msg)) {
            *result = &(*it);
            return true;
        }
    }
    
    return false;
}

void gotoDoorAndSay(Location* location, std::string spoken_text) {
    if(location->hasDoor()) {
        twitcher_actions::FaceDoorGoal face_goal;
        face_goal.door_name = location->getFirstDoor();
        faceDoorClient->sendGoal(face_goal);
        faceDoorClient->waitForResult();
        
    }
    else {
        twitcher_actions::GoToLocationGoal goto_goal;
        goto_goal.location_name = location->getAspName();
        goToLocationClient->sendGoal(goto_goal);
        goToLocationClient->waitForResult();
    }
    
    twitcher_actions::SayGoal say_goal;
    say_goal.message = spoken_text;
    sayClient->sendGoal(say_goal);
    sayClient->waitForResult();
}
