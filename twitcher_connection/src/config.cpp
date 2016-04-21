/*
 * Copyright 2016 <copyright holder> <email>
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

#include "twitcher_connection/config.h"

Config* Config::getInstance() {
    if(_instance == nullptr) {
        _instance = new Config();
    }
    
    return _instance;
}

Config::Config()
    : empty("")
{

}

const std::string& Config::getString(const std::string& name)
{
    auto it = data.find(name);
    if(it == data.end()) {
        return  empty;
    }
    
    return it->second;
}


const std::string& Config::twitterApi() { return getString("twitter_api"); }
const std::string& Config::oauthConsumerKey() { return getString("oauth_consumer_key"); }
const std::string& Config::oauthConsumerSecret() { return getString("oauth_consumer_secret"); }
const std::string& Config::oauthSignatureMethod() { return getString("oauth_signature_method"); }
const std::string& Config::oauthToken() { return getString("oauth_token"); }
const std::string& Config::oauthTokenSecret() { return getString("oauth_token_secret"); }
const std::string& Config::oauthVersion() { return getString("oauth_version"); }


Config::~Config()
{

}
