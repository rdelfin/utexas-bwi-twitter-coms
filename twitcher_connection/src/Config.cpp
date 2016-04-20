#include <twitcher_connection/Config.h>

Config::Config()
{

}

const std::string& Config::operator[](const std::string&)
{

}

const std::string& Config::getTwitterApiUrl()
{

}

const std::string& Config::getOauthConsumerKey()
{

}

const std::string& Config::getOauthConsumerSecret()
{

}

const std::string& Config::getOauthSignatureMethod()
{

}

const std::string& Config::getOauthToken()
{

}

const std::string& Config::getOauthTokenSecret()
{

}

const std::string& Config::getOauthVersion()
{

}




Config* Config::getInstance()
{
    return _instance;
}
