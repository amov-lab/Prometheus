#include "param_manager.hpp"

ParamManager::ParamManager(ros::NodeHandle &nh) : nh_(nh)
{
}

std::vector<std::string> ParamManager::getParamNames()
{
    std::vector<std::string> param_names;
    nh_.getParamNames(param_names);
    return param_names;
}

std::unordered_map<std::string, std::string> ParamManager::getParams(std::string keywords)
{
    std::vector<std::string> param_names = getParamNames();
    std::unordered_map<std::string, std::string> param_map;

    for (const std::string &param_name : param_names)
    {
        // 判断搜索的关键字
        if (!keywords.empty())
        {
            std::string param_name_lower = toLowerCase(param_name);
            std::string keywords_lower = toLowerCase(keywords);
            if (param_name_lower.find(keywords_lower) == std::string::npos)
            {
                continue;
            }
        }

        // 尝试获取不同类型的参数
        std::string value_string;
        if (nh_.getParam(param_name, value_string))
        {
            param_map[param_name] = value_string;
        }
        else
        {
            int int_value;
            if (nh_.getParam(param_name, int_value))
            {
                param_map[param_name] = std::to_string(int_value);
            }
            else
            {
                double double_value;
                if (nh_.getParam(param_name, double_value))
                {
                    param_map[param_name] = std::to_string(double_value);
                }
                else
                {
                    bool bool_value;
                    if (nh_.getParam(param_name, bool_value))
                    {
                        param_map[param_name] = std::to_string(bool_value);
                    }
                    else
                    {
                        std::vector<int> int_vector_value;
                        if (nh_.getParam(param_name, int_vector_value))
                        {
                            std::string vector_string = "[";
                            for (const auto &val : int_vector_value)
                            {
                                vector_string += std::to_string(val) + " ";
                            }
                            vector_string += "]";
                            param_map[param_name] = vector_string;
                        }
                        else
                        {
                            std::vector<double> double_vector_value;
                            if (nh_.getParam(param_name, double_vector_value))
                            {
                                std::string vector_string = "[";
                                for (const auto &val : double_vector_value)
                                {
                                    vector_string += std::to_string(val) + " ";
                                }
                                vector_string += "]";
                                param_map[param_name] = vector_string;
                            }
                            else
                            {
                                std::vector<std::string> string_vector_value;
                                if (nh_.getParam(param_name, string_vector_value))
                                {
                                    std::string vector_string = "[";
                                    for (const auto &val : string_vector_value)
                                    {
                                        vector_string += val + " ";
                                    }
                                    vector_string += "]";
                                    param_map[param_name] = vector_string;
                                }
                                else
                                {
                                    param_map[param_name] = "(failed to get value)";
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // for (const auto& pair : param_map) {
    //     std::cout << pair.first << ": " << pair.second << std::endl;
    // }

    return param_map;
}

std::string ParamManager::toLowerCase(const std::string &str)
{
    std::string result = str; // 创建一个副本
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });
    return result; // 返回转换后的字符串
}

bool ParamManager::setParam(std::string name, std::string value)
{
    // 获取当前参数的值
    XmlRpc::XmlRpcValue param_value;
    if (nh_.getParam(name, param_value))
    {
        // 检查参数的类型并进行相应的设置
        switch (param_value.getType())
        {
        case XmlRpc::XmlRpcValue::TypeInt:
            try
            {
                int int_value = std::stoi(value);
                nh_.setParam(name, int_value);
                //ROS_INFO("Parameter '%s' set to int: %d", name.c_str(), int_value);
            }
            catch (const std::invalid_argument &)
            {
                ROS_WARN("Invalid conversion from string to int for parameter '%s'", name.c_str());
                return false;
            }
            break;

        case XmlRpc::XmlRpcValue::TypeDouble:
            try
            {
                double double_value = std::stod(value);
                nh_.setParam(name, double_value);
                //ROS_INFO("Parameter '%s' set to double: %f", name.c_str(), double_value);
            }
            catch (const std::invalid_argument &)
            {
                ROS_WARN("Invalid conversion from string to double for parameter '%s'", name.c_str());
                return false;
            }
            break;

        case XmlRpc::XmlRpcValue::TypeString:
            nh_.setParam(name, value);
            //ROS_INFO("Parameter '%s' set to string: %s", name.c_str(), value.c_str());
            break;

        case XmlRpc::XmlRpcValue::TypeBoolean:
            if (value == "true" || value == "1")
            {
                nh_.setParam(name, true);
                //ROS_INFO("Parameter '%s' set to boolean: true", name.c_str());
            }
            else if (value == "false" || value == "0")
            {
                nh_.setParam(name, false);
                //ROS_INFO("Parameter '%s' set to boolean: false", name.c_str());
            }
            else
            {
                ROS_WARN("Invalid conversion from string to boolean for parameter '%s'", name.c_str());
                return false;
            }
            break;

        default:
            ROS_WARN("Unsupported parameter type for '%s'", name.c_str());
            return false;
        }
    }
    else
    {
        // 如果参数不存在，直接设置为字符串
        nh_.setParam(name, value);
        //ROS_INFO("Parameter '%s' set to string (default): %s", name.c_str(), value.c_str());
    }
    return true;
}