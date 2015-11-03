/*
 * Copyright 2015 <copyright holder> <email>
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

#include "twitcher_connection/TwitterMentionsMonitor.h"

#include "twitcher_connection/TwitterMentions.h"

TwitterMentionsMonitor::TwitterMentionsMonitor(ros::NodeHandle& nh, 
                                               TwitterRequestHandler handler) : 
                                               lastTweetId(-1), handler(handler)
{
    // Call once and then call every minute
    receiveNewMentions();
    timer = nh.createTimer(ros::Duration(60.0), boost::bind(&TwitterMentionsMonitor::timerCallback, this, _1));
}

void TwitterMentionsMonitor::timerCallback(const ros::TimerEvent te)
{
    receiveNewMentions();
}

void TwitterMentionsMonitor::receiveNewMentions()
{
    TwitterApiCall* call = new TwitterMentions(
        "/home/rdelfin/Documents/twitter_config.json", -1, lastTweetId, -1,
        true, false, false);
    
    std::string result = handler.makeRequest(call);
    
    ROS_INFO("We received the following mentions:");
    ROS_INFO("%s", result.c_str());
}


TwitterMentionsMonitor::~TwitterMentionsMonitor()
{

}
