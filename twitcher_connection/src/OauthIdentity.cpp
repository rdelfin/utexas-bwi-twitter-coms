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

#include "twitcher_connection/OauthIdentity.h"
#include "twitcher_connection/base64.h"

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Easy.hpp>

#include <json/json.h>
#include <json/value.h>

#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <ctime>
#include <cstring>

#include <openssl/hmac.h>

OauthIdentity::OauthIdentity(std::string consumerKey, std::string consumerSecret,
                  std::string accessToken, std::string accessTokenSecret, 
                  std::string apiUrl, std::string version,
                  std::map<std::string, std::string> queryVals,
                  std::string httpMethod)
                  : consumerKey(consumerKey), consumerSecret(consumerSecret),
                    accessToken(accessToken), accessTokenSecret(accessTokenSecret),
                    version(version), apiUrl(apiUrl), queryVals(queryVals),
                    httpMethod(httpMethod)
{
    srand(time(NULL));
}

OauthIdentity::OauthIdentity(std::string configFile, std::string apiUrl, 
                             std::map<std::string, std::string> queryVals,
                             std::string httpMethod)
                            : apiUrl(apiUrl), queryVals(queryVals),
                              httpMethod(httpMethod)
{
    srand(time(NULL));
    Json::Value root;   // 'root' will contain the root value after parsing.
    std::ifstream config_doc(configFile.c_str());
    Json::Reader reader;
    reader.parse(config_doc, root);
    
    // This is the best notation I have ever seen
    consumerKey = root["twitter_oauth"]["oauth_consumer_key"].asString();
    consumerSecret = root["twitter_oauth"]["oauth_signature"].asString();
    accessToken = root["twitter_oauth"]["oauth_token"].asString();
    accessTokenSecret = root["twitter_oauth"]["oauth_version"].asString();
}

std::string OauthIdentity::getAuthHeader()
{
    std::stringstream headerStream;
    
    signRequest();
    
    headerStream << "Authorization: OAuth "
                    "oauth_consumer_key=\"" << consumerKey << 
                    "\", oauth_nonce=\"" << nonce <<
                    "\", oauth_signature=\"" << signature <<
                    "\", oauth_signature_method=\"HMAC-SHA1\", "
                    "oauth_timestamp=\"" << timestamp <<
                    "\", oauth_token=\"" << accessToken <<
                    "\", oauth_version=\"" << version << "\"";
    
    authHeader = headerStream.str();
    
    return authHeader;
}


static char charMap[] = {
                         '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                         'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
                         'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                         'w', 'x', 'y', 'z',
                         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K',
                         'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
                         'W', 'X', 'Y', 'Z'
                        };

void OauthIdentity::generateOauthNonce()
{
    char c;
    std::stringstream resultStream;
    
    // Generates 32 random characters and appends them to the resultStream
    for(int i = 0; i < 32; i++)
    {
        c = charMap[rand() % 62];
        resultStream << c;
    }
    
    nonce = resultStream.str();
}

void OauthIdentity::signRequest()
{
    std::map<std::string, std::string> values;
    std::stringstream parametersStream;
    std::string paramString;
    std::stringstream dataStream;
    std::string dataString;
    std::string signingKey;
    std::stringstream signingKeyStream;
    
    generateOauthNonce();
    this->timestamp = std::string(std::to_string(time(NULL)));
    
    for(std::map<std::string, std::string>::iterator it = queryVals.begin();
        it != queryVals.end(); ++i)
        values[it->first] = curlpp::escape(it->second);
    
    values["oauth_consumer_key"] = consumerKey;
    values["oauth_nonce"] = nonce;
    values["oauth_signature_method"] = "HMAC-SHA1";
    values["oauth_timestamp"] = timestamp;
    values["oauth_token"] = accessToken;
    values["oauth_version"] = version;
    
    
    for(std::map<std::string, std::string>::iterator it = values.begin();
            it != values.end(); ++i) {
        parametersStream << it->first << "=" << it->second << "&"; 
    }
    
    paramString = parametersStream.str();
    paramString.erase(paramString.length() - 1, 1);
    
    dataStream << httpMethod << "&" << curlpp::escape(apiUrl)
               << "&" << curlpp::escape(paramString);
    dataString = dataStream.str();
                 
    signingKeyStream << curlpp::escape(consumerSecret)
                     << "&" << curlpp::escape(accessTokenSecret);
    signingKey  = signingKeyStream.str();
    
    // Generate HMAC-SHA1 key
    unsigned char* hash = HMAC(EVP_sha1(), signingKey.c_str(), signingKey.length(),
         (unsigned char*)dataString.c_str(), dataString.length, NULL, NULL);
    
    //                                    v--SHA1 returns 20 bytes
    this->signature = base64_encode(hash, 20);
}

OauthIdentity::~OauthIdentity()
{

}


