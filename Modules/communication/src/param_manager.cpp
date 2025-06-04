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
            bool keyword_matches = false;

            // 检查是否包含 '|' 符号
            if (keywords_lower.find('|') != std::string::npos)
            {
                // 拆分关键字并检查是否有匹配
                std::istringstream ss(keywords_lower);
                std::string sub_keyword;
                while (std::getline(ss, sub_keyword, '|'))
                {
                    // 去除空白
                    sub_keyword.erase(remove(sub_keyword.begin(), sub_keyword.end(), ' '), sub_keyword.end());
                    if (param_name_lower.find(sub_keyword) != std::string::npos)
                    {
                        keyword_matches = true; // 找到至少一个匹配
                        break; // 找到匹配后跳出拆分循环
                    }
                }
            }
            else
            {
                // 如果没有 '|'，则直接比较
                keyword_matches = (param_name_lower.find(keywords_lower) != std::string::npos);
            }

            // 如果没有匹配，跳过当前参数名
            if (!keyword_matches)
            {
                continue;
            }
        }

        XmlRpc::XmlRpcValue value;
        if (!nh_.getParam(param_name, value)) continue;

        // 检查获取参数的结果并转换为字符串
        if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            param_map[param_name] = std::string(value);
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            param_map[param_name] = std::to_string(static_cast<int>(value));
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            param_map[param_name] = std::to_string(static_cast<double>(value));
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            param_map[param_name] = value ? "true" : "false";
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            std::string vector_string = "[";
            for (int i = 0; i < value.size(); ++i)
            {
                vector_string += XmlRpcValueToString(value[i]);
                if (i < value.size() - 1)
                {
                    vector_string += ", ";
                }
            }
            vector_string += "]";
            param_map[param_name] = vector_string;
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            std::string struct_string = "{";
            for (int i = 0; i < value.size(); ++i)
            {
                struct_string += std::string(value[i][0]) + ": " + XmlRpcValueToString(value[i][1]) + ", ";
            }
            if (value.size() > 0) {
                struct_string.erase(struct_string.length() - 2); // 移除最后的逗号和空格
            }
            struct_string += "}";
            param_map[param_name] = struct_string;
        }
        else
        {
            param_map[param_name] = "(unsupported type)";
        }
    }

    return param_map;
}

std::unordered_map<std::string, std::string> ParamManager::getParams(const std::vector<std::string>& keywords)
{
    std::vector<std::string> param_names = getParamNames();
    std::unordered_map<std::string, std::string> param_map;

    for (const std::string &param_name : param_names)
    {
        // 检查所有关键字
        bool all_keywords_match = true;

        for (const std::string &keyword : keywords)
        {
            if (!keyword.empty())
            {
                std::string param_name_lower = toLowerCase(param_name);
                std::string keyword_lower = toLowerCase(keyword);
                
                bool keyword_matches = false;

                // 检查是否包含 '|'
                if (keyword_lower.find('|') != std::string::npos)
                {
                    // 拆分关键字并检查是否有匹配
                    std::istringstream ss(keyword_lower);
                    std::string sub_keyword;
                    while (std::getline(ss, sub_keyword, '|'))
                    {
                        // 去除空白
                        sub_keyword.erase(remove(sub_keyword.begin(), sub_keyword.end(), ' '), sub_keyword.end());
                        if (param_name_lower.find(sub_keyword) != std::string::npos)
                        {
                            keyword_matches = true; // 找到至少一个匹配
                            break; // 跳出拆分循环
                        }
                    }
                }
                else
                {
                    // 直接检查没有 '|' 的关键字
                    keyword_matches = (param_name_lower.find(keyword_lower) != std::string::npos);
                }

                // 如果当前关键字不匹配，设置标志并跳出循环
                if (!keyword_matches)
                {
                    all_keywords_match = false;
                    break; // 如果有一个关键字不匹配，跳出循环
                }
            }
        }

        // 如果不满足所有关键字，继续下一个参数名
        if (!all_keywords_match)
        {
            continue;
        }

        XmlRpc::XmlRpcValue value;
        if (!nh_.getParam(param_name, value)) continue;

        // 检查获取参数的结果并转换为字符串
        if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            param_map[param_name] = std::string(value);
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            param_map[param_name] = std::to_string(static_cast<int>(value));
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            param_map[param_name] = std::to_string(static_cast<double>(value));
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            param_map[param_name] = value ? "true" : "false";
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            std::string vector_string = "[";
            for (int i = 0; i < value.size(); ++i)
            {
                vector_string += XmlRpcValueToString(value[i]);
                if (i < value.size() - 1)
                {
                    vector_string += ", ";
                }
            }
            vector_string += "]";
            param_map[param_name] = vector_string;
        }
        else if (value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            std::string struct_string = "{";
            for (int i = 0; i < value.size(); ++i)
            {
                struct_string += std::string(value[i][0]) + ": " + XmlRpcValueToString(value[i][1]) + ", ";
            }
            if (value.size() > 0) {
                struct_string.erase(struct_string.length() - 2); // 移除最后的逗号和空格
            }
            struct_string += "}";
            param_map[param_name] = struct_string;
        }
        else
        {
            param_map[param_name] = "(unsupported type)";
        }
    }

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

// XmlRpcValue 转换为 std::string 的辅助函数
std::string ParamManager::XmlRpcValueToString(const XmlRpc::XmlRpcValue& value)
{
    switch (value.getType()) {
        case XmlRpc::XmlRpcValue::TypeString:
            return std::string(value);
        case XmlRpc::XmlRpcValue::TypeInt:
            return std::to_string(static_cast<int>(value));
        case XmlRpc::XmlRpcValue::TypeDouble:
            return std::to_string(static_cast<double>(value));
        case XmlRpc::XmlRpcValue::TypeBoolean:
            return value ? "true" : "false";
        default:
            return "(unsupported type)";
    }
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
                // ROS_INFO("Parameter '%s' set to int: %d", name.c_str(), int_value);
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
                // ROS_INFO("Parameter '%s' set to double: %f", name.c_str(), double_value);
            }
            catch (const std::invalid_argument &)
            {
                ROS_WARN("Invalid conversion from string to double for parameter '%s'", name.c_str());
                return false;
            }
            break;

        case XmlRpc::XmlRpcValue::TypeString:
            nh_.setParam(name, value);
            // ROS_INFO("Parameter '%s' set to string: %s", name.c_str(), value.c_str());
            break;

        case XmlRpc::XmlRpcValue::TypeBoolean:
            if (value == "true" || value == "1")
            {
                nh_.setParam(name, true);
                // ROS_INFO("Parameter '%s' set to boolean: true", name.c_str());
            }
            else if (value == "false" || value == "0")
            {
                nh_.setParam(name, false);
                // ROS_INFO("Parameter '%s' set to boolean: false", name.c_str());
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
        // ROS_INFO("Parameter '%s' set to string (default): %s", name.c_str(), value.c_str());
    }
    return true;
}