/*
 * Copyright 2016 Ricardo Delfin Garcia <ricardo.delfin@utexas.edu>
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

#pragma once

#include <string>
#include <unordered_map>

class Config
{
public:
    Config();
    
    // Get property from the map of objects
    const std::string& getString(const std::string& name);
    
    // List of all objects
    const std::string& twitterApi();
    const std::string& oauthConsumerKey();
    const std::string& oauthConsumerSecret();
    const std::string& oauthSignatureMethod();
    const std::string& oauthToken();
    const std::string& oauthTokenSecret();
    const std::string& oauthVersion();
    
    ~Config();
    
    // MORE SINGLETON
    static Config* getInstance();
    
private:
    std::unordered_map<std::string, std::string> data;
    std::string empty;
    
    // Singleton
    static Config* _instance;
};
