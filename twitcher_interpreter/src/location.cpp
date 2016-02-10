/*
 * Copyright 2016 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
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

#include <twitcher_interpreter/location.h>
#include <json/json.hpp>
#include <string>

using json = nlohmann::json;

Location::Location()
{

}

Location::Location(const std::string &name, const std::vector<std::string> &common_name)
  : asp_name(name), common_name(common_name)
{
    
}

Location::Location(std::string json_str)
{
    json root = json::parse(json_str);
    
    asp_name = root["asp_name"];
    
    for(int i = 0; i < root["common_names"].size(); i++) {
	common_name.push_back(root["common_names"][i]);
    }
}

bool Location::isMentioned(std::string tweet)
{
  for(auto it = common_name.begin(); it != common_name.end(); ++it)
    if(tweet.find(*it) != std::string::npos)
	return true;
  
  return false;
}

std::string Location::get_asp_name() {
    return this->asp_name;
}

Location::~Location()
{
    
}
