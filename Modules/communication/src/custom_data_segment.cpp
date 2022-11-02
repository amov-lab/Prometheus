#include "custom_data_segment.hpp"

CustomDataSegment::CustomDataSegment()
{
    
}
CustomDataSegment::CustomDataSegment(struct CustomDataSegment_1 datas)
{
    datas_ = datas;
}

void CustomDataSegment::initCommunication(int id, int udp_port, int tcp_port)
{
    communication_.init(id,udp_port,tcp_port,25555);
}

void CustomDataSegment::initCommunication(Communication communication)
{
    communication_ = communication;
}

bool CustomDataSegment::addValue(std::string name, BasicDataTypeAndValue::Type type, std::string value)
{
    int index = indexof(name);
    if (index == -1)
    {
        BasicDataTypeAndValue data;
        data.name = name;
        data.type = type;
        data.value = value;
        datas_.datas.push_back(data);
        return true;
    }
    return false;
}

int CustomDataSegment::indexof(std::string name)
{
    for (int i = 0; i < datas_.datas.size(); i++)
    {
        if (datas_.datas[i].name == name)
        {
            return i;
        }
    }
    return -1;
}

void CustomDataSegment::setValue(std::string name, int value)
{
    int index = indexof(name);
    if (index != -1)
    {
        datas_.datas[index].type = BasicDataTypeAndValue::INTEGER;
        datas_.datas[index].value = to_string(value);
    }
    else
    {
        addValue(name, BasicDataTypeAndValue::INTEGER, to_string(value));
    }
}
void CustomDataSegment::setValue(std::string name, float value)
{
    int index = indexof(name);
    if (index != -1)
    {
        datas_.datas[index].type = BasicDataTypeAndValue::FLOAT;
        datas_.datas[index].value = to_string(value);
    }
    else
    {
        addValue(name, BasicDataTypeAndValue::FLOAT, to_string(value));
    }
}
void CustomDataSegment::setValue(std::string name, double value)
{
    int index = indexof(name);
    if (index != -1)
    {
        datas_.datas[index].type = BasicDataTypeAndValue::DOUBLE;
        datas_.datas[index].value = to_string(value);
    }
    else
    {
        addValue(name, BasicDataTypeAndValue::DOUBLE, to_string(value));
    }
}
void CustomDataSegment::setValue(std::string name, bool value)
{
    int index = indexof(name);
    if (index != -1)
    {
        datas_.datas[index].type = BasicDataTypeAndValue::BOOLEAN;
        datas_.datas[index].value = value ? "true" : "false";
    }
    else
    {
        addValue(name, BasicDataTypeAndValue::BOOLEAN, value ? "true" : "false");
    }
}
void CustomDataSegment::setValue(std::string name, std::string value)
{
    int index = indexof(name);
    if (index != -1)
    {
        datas_.datas[index].type = BasicDataTypeAndValue::STRING;
        datas_.datas[index].value = value;
    }
    else
    {
        addValue(name, BasicDataTypeAndValue::STRING, value);
    }
}
void CustomDataSegment::setValue(std::string name, char *value)
{
    setValue(name, std::string(value));
}

bool CustomDataSegment::getValue(std::string name, int &value)
{
    int index = indexof(name);
    if (index != -1)
    {
        if (datas_.datas[index].type == BasicDataTypeAndValue::INTEGER)
        {
            value = atoi(datas_.datas[index].value.c_str());
            return true;
        }
    }
    return false;
}
bool CustomDataSegment::getValue(std::string name, float &value)
{
    int index = indexof(name);
    if (index != -1)
    {
        if (datas_.datas[index].type == BasicDataTypeAndValue::FLOAT || datas_.datas[index].type == BasicDataTypeAndValue::DOUBLE)
        {
            value = atof(datas_.datas[index].value.c_str());
            return true;
        }
    }
    return false;
}
bool CustomDataSegment::getValue(std::string name, double &value)
{
    int index = indexof(name);
    if (index != -1)
    {
        if (datas_.datas[index].type == BasicDataTypeAndValue::DOUBLE || datas_.datas[index].type == BasicDataTypeAndValue::FLOAT)
        {
            value = stod(datas_.datas[index].value.c_str());
            return true;
        }
    }
    return false;
}
bool CustomDataSegment::getValue(std::string name, bool &value)
{
    int index = indexof(name);
    if (index != -1)
    {
        if (datas_.datas[index].type == BasicDataTypeAndValue::BOOLEAN)
        {
            value = datas_.datas[index].value == "true" ? true : false;
            return true;
        }
    }
    return false;
}
bool CustomDataSegment::getValue(std::string name, std::string &value)
{
    int index = indexof(name);
    if (index != -1)
    {
        if (datas_.datas[index].type == BasicDataTypeAndValue::STRING)
        {
            value = datas_.datas[index].value;
            return true;
        }
    }
    return false;
}

struct CustomDataSegment_1 CustomDataSegment::getCustomDataSegment()
{
    return datas_;
}

void CustomDataSegment::sendUdp(std::string udp_ip)
{
    communication_.sendMsgByUdp(communication_.encodeMsg(Send_Mode::UDP,datas_),udp_ip);
}

void CustomDataSegment::sendTcp(std::string tcp_ip)
{
    communication_.sendMsgByUdp(communication_.encodeMsg(Send_Mode::TCP,datas_),tcp_ip);
}