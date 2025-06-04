#ifndef PARAM_MANAGER_HPP
#define PARAM_MANAGER_HPP

#include <ros/ros.h>
#include <string>
#include <vector>
#include <variant>
#include <unordered_map>
#include <algorithm> // std::transform
#include <cctype>    // std::tolower
#include <sstream>

class ParamManager{
public:
    ParamManager(ros::NodeHandle &nh);

    std::vector<std::string> getParamNames();

    std::unordered_map<std::string, std::string> getParams(std::string keywords = "");

    std::unordered_map<std::string, std::string> getParams(const std::vector<std::string>& keywords);

    bool setParam(std::string name,std::string value);
private:
    std::string toLowerCase(const std::string &str);

    std::string XmlRpcValueToString(const XmlRpc::XmlRpcValue& value);

private:
    ros::NodeHandle nh_;
};

#endif