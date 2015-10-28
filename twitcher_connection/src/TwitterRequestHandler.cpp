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

#include "twitcher_connection/TwitterRequestHandler.h"

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Easy.hpp>

#include <json/json.h>
#include <json/value.h>
#include <fstream>

#include <cstdlib>
#include <sstream>
#include <ctime>
#include <iostream>

#include <ros/ros.h>

TwitterRequestHandler::TwitterRequestHandler()
    : configFile("~/Documents/twitter_config.json")
{
    srand(time(NULL));
    
    Json::Value root;   // 'root' will contain the root value after parsing.
    std::ifstream config_doc(configFile.c_str());
    Json::Reader reader;
    reader.parse(config_doc, root);
    
    // This is the best notation I have ever seen
    oauthConsumerKey = root["twitter_oauth"]["oauth_consumer_key"].asString();
    oauthSignature = root["twitter_oauth"]["oauth_signature"].asString();
    oauthSignatureMethod = root["twitter_oauth"]["oauth_signature_method"].asString();
    oauthToken = root["twitter_oauth"]["oauth_token"].asString();
    oauthVersion = root["twitter_oauth"]["oauth_version"].asString();
    apiUrl = root["twitter_api_url"].asString();
    
    ROS_INFO("Hello again!");
}

TwitterRequestHandler::TwitterRequestHandler(const TwitterRequestHandler& other)
{
    this->configFile = other.configFile;
    this->oauthToken = other.oauthToken;
    this->oauthSignature = other.oauthSignature;
    this->oauthConsumerKey = other.oauthConsumerKey;
    this->oauthVersion = other.oauthVersion;
    this->apiUrl = other.apiUrl;
}

static char charMap[] = {
                         '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                         'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
                         'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x',
                         'y', 'z',
                         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
                         'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                         'Y', 'Z'
                        };

std::string generateOauthNonce()
{
    char c;
    std::stringstream resultStream;
    
    // Generates 32 random characters and appends them to the resultStream
    for(int i = 0; i < 32; i++)
    {
        c = charMap[rand() % 62];
        resultStream << c;
    }
    
    return resultStream.str();
}

TwitterRequestHandler::~TwitterRequestHandler()
{

}
