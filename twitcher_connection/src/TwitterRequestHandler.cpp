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
#include "twitcher_connection/OauthIdentity.h"

#include <json/json.h>
#include <json/value.h>

#include <sstream>
#include <ctime>
#include <iostream>

#include <ros/ros.h>

TwitterRequestHandler::TwitterRequestHandler()
    : configFile("~/Documents/twitter_config.json")
{
    Json::Value root;   // 'root' will contain the root value after parsing.
    std::ifstream config_doc(configFile.c_str());
    Json::Reader reader;
    reader.parse(config_doc, root);
    
    apiUrl = root["twitter_api_url"].asString();
}

TwitterRequestHandler::TwitterRequestHandler(const TwitterRequestHandler& other)
    : authorizationHeader(other.authorizationHeader), apiUrl(other.apiUrl),
      configFile(other.configFile)
{
    
}



TwitterRequestHandler::~TwitterRequestHandler()
{

}
