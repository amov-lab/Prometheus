#ifndef CUSTOM_DATA_SEGMENT_HPP
#define CUSTOM_DATA_SEGMENT_HPP

#include "Struct.hpp"
#include "string.h"
#include "communication.hpp"

using namespace std;

class CustomDataSegment
{
public:
    CustomDataSegment();
    CustomDataSegment(struct CustomDataSegment_1);

    void initCommunication(int id, int udp_port, int tcp_port);
    void initCommunication(Communication communication);

    int indexof(std::string name);
    bool addValue(std::string name, BasicDataTypeAndValue::Type type, std::string value);

    void setValue(std::string name, int value);
    void setValue(std::string name, float value);
    void setValue(std::string name, double value);
    void setValue(std::string name, bool value);
    void setValue(std::string name, std::string value);
    void setValue(std::string name, char* value);

    bool getValue(std::string name, int &value);
    bool getValue(std::string name, float &value);
    bool getValue(std::string name, double &value);
    bool getValue(std::string name, bool &value);
    bool getValue(std::string name, std::string &value);

    struct CustomDataSegment_1 getCustomDataSegment();

    void sendUdp(std::string udp_ip);
    void sendTcp(std::string tcp_ip);
private:
    struct CustomDataSegment_1 datas_;
    Communication communication_;
};

#endif