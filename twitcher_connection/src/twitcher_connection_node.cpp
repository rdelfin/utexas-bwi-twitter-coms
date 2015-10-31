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

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_connectiwhaon_node");
    
    TwitterRequestHandler handler;
    
    TwitterApiCall* api = new TwitterMentions("/home/rdelfin/Documents/twitter_config.json",
                                              -1, -1, -1, false, true, true);
    
    std::string result = handler.makeRequest(api);
    
    ROS_INFO("We got dis!\n%s", result.c_str());
    
}