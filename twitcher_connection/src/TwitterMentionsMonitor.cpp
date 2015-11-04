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

#include <json/json.h>

#include "twitcher_connection/TwitterMentionsMonitor.h"
#include "twitcher_connection/TwitterMentions.h"

#include "twitcher_connection/Tweet.h"

TwitterMentionsMonitor::TwitterMentionsMonitor(ros::NodeHandle& nh, 
                                               TwitterRequestHandler handler) : 
                                               lastTweetId(-1), handler(handler)
{
    /* Call the timer callback every minute. This will cause the program to
     * process all the tweets and send them as messages through a topic */
    timer = nh.createTimer(ros::Duration(60.0), boost::bind(&TwitterMentionsMonitor::timerCallback, this, _1));
    mention_publisher = nh.advertise<twitcher_connection::Tweet>("twitter_mentions", 1000);
}

void TwitterMentionsMonitor::timerCallback(const ros::TimerEvent te)
{
    receiveNewMentions();
}

void TwitterMentionsMonitor::receiveNewMentions()
{
    TwitterApiCall* call = new TwitterMentions(
        "/home/rdelfin/Documents/twitter_config.json", -1, lastTweetId, -1,
        false, false, false);
    
    std::string result = handler.makeRequest(call);
    
    Json::Value root;
    Json::Reader reader;
    reader.parse(result, root);
    
    // Set the last tweet ID to the 0th index in the array (always the latest)
    lastTweetId = root[0]["id"].asInt64();
    
    // Iterate in reverse temporal order
    for(int i = root.size() - 1; i >= 0; i--) {
        twitcher_connection::Tweet tweet;
        tweet.id = root[(int)i]["id"].asInt64();
        tweet.message = root[(int)i]["text"].asString();
        tweet.sender = root[(int)i]["user"]["id_str"].asString();
        
        mention_publisher.publish<twitcher_connection::Tweet>(tweet);
    }
    
    delete call;
    
    ROS_INFO("We received the following mentions:");
    ROS_INFO("%s", result.c_str());
}


TwitterMentionsMonitor::~TwitterMentionsMonitor()
{
    
}
