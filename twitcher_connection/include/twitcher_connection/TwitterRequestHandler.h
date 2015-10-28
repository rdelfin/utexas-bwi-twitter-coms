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

#ifndef TWITTERREQUESTHANDLER_H
#define TWITTERREQUESTHANDLER_H

#include <string>

class TwitterRequestHandler
{
public:
    TwitterRequestHandler();
    TwitterRequestHandler(const TwitterRequestHandler& other);
    ~TwitterRequestHandler();
    
    static TwitterRequestHandler& getInstance() { return _instance; }
    
private:
    std::string configFile;
    
    std::string oauthToken;
    std::string oauthSignature;
    std::string oauthConsumerKey;
    std::string apiUrl;
    std::string oauthVersion;
    std::string oauthSignatureMethod;
    
    static std::string generateOauthNonce();
    
    static TwitterRequestHandler _instance;
};

#endif // TWITTERREQUESTHANDLER_H