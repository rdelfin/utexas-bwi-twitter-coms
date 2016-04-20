#pragma once
#include <dynamic_reconfigure/server.h>

#include <twitcher_connection/configConfig.h>
#include <twitcher_connection/oauthConfig.h>

#include <string>
#include <vector>

class Config {
public:
    Config();
    
    // Data getters
    const std::string& operator[](const std::string&);
    const std::string& getTwitterApiUrl();
    const std::string& getOauthConsumerKey();
    const std::string& getOauthConsumerSecret();
    const std::string& getOauthSignatureMethod();
    const std::string& getOauthToken();
    const std::string& getOauthTokenSecret();
    const std::string& getOauthVersion();
    
    ~Config();
    
    // Static members
    static Config* getInstance();
private:
    static Config* _instance = NULL;
    
    std::vector<std::string> data;
    
    dynamic_reconfigure::Server<twitcher>
};