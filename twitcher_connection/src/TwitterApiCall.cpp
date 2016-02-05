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

#include "twitcher_connection/TwitterApiCall.h"

#include "json/json.hpp"

#include <fstream>

using json = nlohmann::json;

TwitterApiCall::TwitterApiCall(std::string configFile)
{
    json root;   // 'root' will contain the root value after parsing.
    std::ifstream config_doc(configFile.c_str());
    
    config_doc >> root;
    
    // This is the best notation I have ever seen
    url = root["twitter_api_url"];
}