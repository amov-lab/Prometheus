#ifndef CUSTOM_DATA_SEGMENT_MSG_HPP
#define CUSTOM_DATA_SEGMENT_MSG_HPP

#include "prometheus_msgs/BasicDataTypeAndValue.h"
#include "prometheus_msgs/CustomDataSegment.h"
#include "string.h"

using namespace std;

class CustomDataSegmentMSG
{
public:
    CustomDataSegmentMSG(prometheus_msgs::CustomDataSegment msg)
    {
        custom_data_segment = msg;
    }

    // 返回下标
    int indexof(std::string name)
    {
        int size = custom_data_segment.datas.size();
        for (int i = 0; i < size; i++)
        {
            if (custom_data_segment.datas[i].name == name)
            {
                return i;
            }
        }
        return -1;
    }

    // 添加变量
    bool addValue(std::string name, int value)
    {
        if(indexof(name) == -1)
        {
            prometheus_msgs::BasicDataTypeAndValue data = initBasicDataTypeAndValue(name);
            data.type = prometheus_msgs::BasicDataTypeAndValue::INTEGER;
            data.integer_value = value;
            custom_data_segment.datas.push_back(data);
            return true;
        }
        return false;
    }
    bool addValue(std::string name, bool value)
    {
        if(indexof(name) == -1)
        {
            prometheus_msgs::BasicDataTypeAndValue data = initBasicDataTypeAndValue(name);
            data.type = prometheus_msgs::BasicDataTypeAndValue::BOOLEAN;
            data.boolean_value = value;
            custom_data_segment.datas.push_back(data);
            return true;
        }
        return false;
    }
    bool addValue(std::string name, double value)
    {
        if(indexof(name) == -1)
        {
            prometheus_msgs::BasicDataTypeAndValue data = initBasicDataTypeAndValue(name);
            data.type = prometheus_msgs::BasicDataTypeAndValue::DOUBLE;
            data.double_value = value;
            custom_data_segment.datas.push_back(data);
            return true;
        }
        return false;
    }
    bool addValue(std::string name, float value)
    {
        if(indexof(name) == -1)
        {
            prometheus_msgs::BasicDataTypeAndValue data = initBasicDataTypeAndValue(name);
            data.type = prometheus_msgs::BasicDataTypeAndValue::FLOAT;
            data.float_value = value;
            custom_data_segment.datas.push_back(data);
            return true;
        }
        return false;
    }
    bool addValue(std::string name, string value)
    {
        if(indexof(name) == -1)
        {
            prometheus_msgs::BasicDataTypeAndValue data = initBasicDataTypeAndValue(name);
            data.type = prometheus_msgs::BasicDataTypeAndValue::STRING;
            data.string_value = value;
            custom_data_segment.datas.push_back(data);
            return true;
        }
        return false;
    }
    bool addValue(std::string name, char* value)
    {
        addValue(name,std::string(value));
    }
    
    // 删除变量
    bool deleteValue(std::string name)
    {
        int index = indexof(name);
        if(index == -1) return false;
        custom_data_segment.datas.erase(custom_data_segment.datas.begin() + index);
        return true;
    }

    // 设置变量
    void setValue(std::string name, int value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            custom_data_segment.datas[index].type = prometheus_msgs::BasicDataTypeAndValue::INTEGER;
            custom_data_segment.datas[index].integer_value = value;
        }
        else
        {
            addValue(name,value);
        }
    }
    void setValue(std::string name, bool value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            custom_data_segment.datas[index].type = prometheus_msgs::BasicDataTypeAndValue::BOOLEAN;
            custom_data_segment.datas[index].boolean_value = value;
        }
        else
        {
            addValue(name,value);
        }
    }
    void setValue(std::string name, float value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            custom_data_segment.datas[index].type = prometheus_msgs::BasicDataTypeAndValue::FLOAT;
            custom_data_segment.datas[index].float_value = value;
        }
        else
        {
            addValue(name,value);
        }
    }
    void setValue(std::string name, double value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            custom_data_segment.datas[index].type = prometheus_msgs::BasicDataTypeAndValue::DOUBLE;
            custom_data_segment.datas[index].double_value = value;
        }
        else
        {
            addValue(name,value);
        }
    }
    void setValue(std::string name, std::string value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            custom_data_segment.datas[index].type = prometheus_msgs::BasicDataTypeAndValue::STRING;
            custom_data_segment.datas[index].string_value = value;
        }
        else
        {
            addValue(name,value);
        }
    }
    void setValue(std::string name, char *value)
    {
        setValue(name,std::string(value));
    }

    // 读取变量
    bool getValue(std::string name, int &value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            if (custom_data_segment.datas[index].type == prometheus_msgs::BasicDataTypeAndValue::INTEGER)
            {
                value = custom_data_segment.datas[index].integer_value;
                return true;
            }
        }
        return false;
    }
    bool getValue(std::string name, float &value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            if (custom_data_segment.datas[index].type == prometheus_msgs::BasicDataTypeAndValue::FLOAT)
            {
                value = custom_data_segment.datas[index].float_value;
                return true;
            }
        }
        return false;
    }
    bool getValue(std::string name, double &value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            if (custom_data_segment.datas[index].type == prometheus_msgs::BasicDataTypeAndValue::DOUBLE)
            {
                value = custom_data_segment.datas[index].double_value;
                return true;
            }
        }
        return false;
    }
    bool getValue(std::string name, bool &value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            if (custom_data_segment.datas[index].type == prometheus_msgs::BasicDataTypeAndValue::BOOLEAN)
            {
                value = custom_data_segment.datas[index].boolean_value;
                return true;
            }
        }
        return false;
    }
    bool getValue(std::string name, std::string &value)
    {
        int index = indexof(name);
        if (index != -1)
        {
            if (custom_data_segment.datas[index].type == prometheus_msgs::BasicDataTypeAndValue::STRING)
            {
                value = custom_data_segment.datas[index].string_value;
                return true;
            }
        }
        return false;
    }

    // 返回自定义数据
    prometheus_msgs::CustomDataSegment getCustomDataSegment()
    {
        return custom_data_segment;
    }

    // 初始化基本变量
    prometheus_msgs::BasicDataTypeAndValue initBasicDataTypeAndValue(string name)
    {
        prometheus_msgs::BasicDataTypeAndValue data;
        data.name = name;
        data.integer_value = std::numeric_limits<float>::quiet_NaN();
        data.boolean_value = false;
        data.float_value = std::numeric_limits<float>::quiet_NaN();
        data.double_value = std::numeric_limits<float>::quiet_NaN();
        data.string_value = "";
        return data;
    } 

private:
    prometheus_msgs::CustomDataSegment custom_data_segment;
};

#endif