// 切换定位源节点
#include <ros/ros.h>
#include <iostream>
#include <prometheus_msgs/SwitchLocationSource.h>
#include <prometheus_msgs/GetLocationSource.h>
#include <prometheus_msgs/StartScript.h>
#include <prometheus_msgs/ParamSettings.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/TextInfo.h>
#include "param_manager.hpp"

struct LocationSourceParam
{
    std::string location_source_name;
    std::string start_script;
    std::string close_start_script;
    int start_script_flag;

    std::string location_source;
    std::string takeoff_height;
    std::string maximum_safe_vel_xy;
    std::string maximum_safe_vel_z;
    std::string maximum_vel_error_for_vision;
    std::string maximum_vel_error_for_Odom;
    std::string geo_fence_x_min;
    std::string geo_fence_x_max;
    std::string geo_fence_y_min;
    std::string geo_fence_y_max;
    std::string geo_fence_z_min;
    std::string geo_fence_z_max;

    std::map<std::string, std::string> px4_params;
};

class SwitchLocationSource
{
public:
    SwitchLocationSource(ros::NodeHandle& nh)
    {
        nh.param<int>("uav_id", uav_id, 1);
        param_manager = new ParamManager(nh);
        //【服务】切换定位源
        switch_location_source_server = nh.advertiseService("/uav" + std::to_string(uav_id) + "/prometheus/switch_location_source", &SwitchLocationSource::switchLocationSourceServerCallback, this);
        //【服务】获取支持的定位源
        get_location_source_server = nh.advertiseService("/uav" + std::to_string(uav_id) + "/prometheus/get_location_source", &SwitchLocationSource::getLocationSourceServerCallback, this);
        //【话题】订阅无人机状态
        uav_state_sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/prometheus/state", 10, &SwitchLocationSource::stateCb, this);
        //【话题】发布消息反馈
        text_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(uav_id) + "/prometheus/text_info", 10);
        //【话题】发布参数修改记录
        param_settings_pub = nh.advertise<prometheus_msgs::ParamSettings>("/uav" + std::to_string(uav_id) + "/prometheus/param_settings", 10);
        //【话题】发布命令
        load_cmd_pub = nh.advertise<prometheus_msgs::StartScript>("/uav" + std::to_string(uav_id) + "/prometheus/load_cmd", 10);
        // 初始化
        init();
    };  

    void init()
    {
        // 获取定位源
        std::string node_name = ros::this_node::getName();
        std::unordered_map<std::string, std::string> param_map = param_manager->getParams(node_name + "/Location_Source");
        // 遍历参数，查找定位源
        for (const auto &pair : param_map)
        {
            if(pair.first.find("/control/location_source") != std::string::npos){
                try
                {
                    int location_source = std::stoi(pair.second);
                    location_source_params.insert({location_source, LocationSourceParam()});
                    std::stringstream ss(pair.first);
                    std::string location_source_name;
                    for(int i = 0; i < 4; i++){
                        std::getline(ss, location_source_name, '/');
                    }
                    location_source_params[location_source].location_source = std::to_string(location_source);
                    location_source_params[location_source].location_source_name = location_source_name;
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
            }
        }
        // 遍历已知定位源
        for (const auto &pair : location_source_params)
        {
            int location_source = pair.first;
            std::string location_source_name = pair.second.location_source_name;

            std::string prefix = node_name + "/Location_Source/" + location_source_name;
            std::unordered_map<std::string, std::string> params = param_manager->getParams(prefix);
            // 给其他参数赋值
            for (const auto &param : params)
            {
                std::string param_name = param.first;
                std::string param_value = param.second;
                
                if(param_name.find("closeStartScript") != std::string::npos){
                    location_source_params[location_source].close_start_script = param_value;
                }else if(param_name.find("startScriptFlag") != std::string::npos){
                    try{
                        location_source_params[location_source].start_script_flag = std::stoi(param_value);
                    }
                    catch(const std::exception& e){
                        std::cerr << e.what() << '\n';
                        location_source_params[location_source].start_script_flag = 1;
                    }
                }else if(param_name.find("startScript") != std::string::npos){
                    location_source_params[location_source].start_script = param_value;
                }else if(param_name.find("control/Takeoff_height") != std::string::npos){
                    location_source_params[location_source].takeoff_height = param_value;
                }else if(param_name.find("control/maximum_safe_vel_xy") != std::string::npos){
                    location_source_params[location_source].maximum_safe_vel_xy = param_value;
                }else if(param_name.find("control/maximum_safe_vel_z") != std::string::npos){
                    location_source_params[location_source].maximum_safe_vel_z = param_value;
                }else if(param_name.find("control/maximum_vel_error_for_vision") != std::string::npos){
                    location_source_params[location_source].maximum_vel_error_for_vision = param_value;
                }else if(param_name.find("control/maximum_vel_error_for_Odom") != std::string::npos){
                    location_source_params[location_source].maximum_vel_error_for_Odom = param_value;
                }else if(param_name.find("geo_fence/x_min") != std::string::npos){
                    location_source_params[location_source].geo_fence_x_min = param_value;
                }else if(param_name.find("geo_fence/x_max") != std::string::npos){
                    location_source_params[location_source].geo_fence_x_max = param_value;
                }else if(param_name.find("geo_fence/y_min") != std::string::npos){
                    location_source_params[location_source].geo_fence_y_min = param_value;
                }else if(param_name.find("geo_fence/y_max") != std::string::npos){
                    location_source_params[location_source].geo_fence_y_max = param_value;
                }else if(param_name.find("geo_fence/z_min") != std::string::npos){
                    location_source_params[location_source].geo_fence_z_min = param_value;
                }else if(param_name.find("geo_fence/z_max") != std::string::npos){
                    location_source_params[location_source].geo_fence_z_max = param_value;
                }else if(param_name.find("px4_params") != std::string::npos){
                    size_t pos = param_name.find_last_of('/');
                    std::string px4_param_name = param_name.substr(pos + 1);
                    location_source_params[location_source].px4_params.insert({px4_param_name, param_value});
                }
            }
        }
    }

    //【服务】切换定位源
    bool switchLocationSourceServerCallback(prometheus_msgs::SwitchLocationSource::Request &req, prometheus_msgs::SwitchLocationSource::Response &res)
    {
        res.success = false;
        int message_type = prometheus_msgs::TextInfo::INFO;
        int location_source = req.location_source; // 定位源标识
        // 前置条件：没有解锁并且连接PX4并且在预设中存在该定位源情况下
        bool preconditions = (!uav_state.armed && uav_state.connected && location_source_params.find(location_source) != location_source_params.end());
        // bool preconditions = true;
        if(preconditions){
            // 获取预设参数值
            struct LocationSourceParam preset_location_source_param = location_source_params[location_source];
            // 启动对于驱动程序 话题不好回环 需要考虑
            // 怎么启动？同时需要进行判断防止重复启动
            prometheus_msgs::StartScript start_script;
            start_script.close_cmd = preset_location_source_param.close_start_script;
            start_script.cmd = preset_location_source_param.start_script;
            start_script.cmd_level = prometheus_msgs::StartScript::LOCATION_SOURCE_CMD;
            start_script.detection_cmd = prometheus_msgs::StartScript::NOT_DETECTION;
            start_script.flag = preset_location_source_param.start_script_flag;
            load_cmd_pub.publish(start_script);

            // 获取uav_control的节点名
            std::string uav_control_node_name = "/uav_control_main_" + std::to_string(uav_id);
            prometheus_msgs::ParamSettings param_settings;
            // 预设默认参数
            param_settings.param_name = {uav_control_node_name + "/control/Takeoff_height", 
                uav_control_node_name + "/control/location_source", uav_control_node_name + "/control/maximum_safe_vel_xy", 
                uav_control_node_name + "/control/maximum_safe_vel_z", uav_control_node_name + "/control/maximum_vel_error_for_vision",
                uav_control_node_name + "/control/maximum_vel_error_for_Odom", uav_control_node_name + "/geo_fence/x_min",
                uav_control_node_name + "/geo_fence/x_max", uav_control_node_name + "/geo_fence/y_min",
                uav_control_node_name + "/geo_fence/y_max", uav_control_node_name + "/geo_fence/z_min",
                uav_control_node_name + "/geo_fence/z_max"
            };
            param_settings.param_value = {preset_location_source_param.takeoff_height, 
                preset_location_source_param.location_source, preset_location_source_param.maximum_safe_vel_xy,
                preset_location_source_param.maximum_safe_vel_z, preset_location_source_param.maximum_vel_error_for_vision,
                preset_location_source_param.maximum_vel_error_for_Odom, preset_location_source_param.geo_fence_x_min,
                preset_location_source_param.geo_fence_x_max, preset_location_source_param.geo_fence_y_min,
                preset_location_source_param.geo_fence_y_max, preset_location_source_param.geo_fence_z_min,
                preset_location_source_param.geo_fence_z_max
            };
            // PX4参数预设值
            for (const auto &param : preset_location_source_param.px4_params)
            {
                param_settings.param_name.push_back(uav_control_node_name + "/px4_params/" + param.first);
                param_settings.param_value.push_back(param.second);
            }
            // 加载PX4参数
            param_settings.param_name.push_back(uav_control_node_name + "/enable_px4_params_load");
            param_settings.param_value.push_back("true");
            // 发布话题
            param_settings_pub.publish(param_settings);
            res.success = true;
            res.message = "Switching location source success: " + preset_location_source_param.location_source_name;
        }else
        {
            message_type = prometheus_msgs::TextInfo::ERROR;
            // 判断无法切换的原因
            if(uav_state.armed){
                res.message = "Switching location source failed: Cannot switch location source operation in unlocked state!"; 
            }else if(!uav_state.connected){
                res.message = "Switching location source failed: Cannot switch location source in PX4 unconnected!";
            }else if(location_source_params.find(location_source) == location_source_params.end()){
                res.message = "Switching location source failed: No relevant parameters found for the preset location source!";
            }
            ROS_ERROR(res.message.c_str());
        }
        // 发布消息反馈
        textInfoPub(message_type, res.message);
        return true;
    }

    //【服务】获取支持的定位源
    bool getLocationSourceServerCallback(prometheus_msgs::GetLocationSource::Request &req, prometheus_msgs::GetLocationSource::Response &res)
    {
        res.success = !location_source_params.empty();
        for (const auto &pair : location_source_params)
        {
            res.location_source.push_back(pair.first);
            res.location_source_name.push_back(pair.second.location_source_name);
        }
        if(!res.success){
            res.message = ros::this_node::getName() + ": not found preset location source";
            // 发布消息反馈
            textInfoPub(prometheus_msgs::TextInfo::WARN, res.message);
        }
        return true;
    }

    //【话题】订阅无人机状态
    void stateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
    {
        uav_state = *msg;
    }

    // 发布消息反馈
    void textInfoPub(int message_type, std::string message)
    {
        prometheus_msgs::TextInfo text_info;
        text_info.header.stamp = ros::Time::now();
        text_info.MessageType = message_type;
        text_info.Message = message;
        text_info_pub.publish(text_info);
    }

private:
    ParamManager* param_manager = nullptr;
    int uav_id;

    //【服务】
    ros::ServiceServer switch_location_source_server;
    ros::ServiceServer get_location_source_server;
    //【订阅话题】
    ros::Subscriber uav_state_sub;
    //【发布话题】
    ros::Publisher text_info_pub;
    ros::Publisher param_settings_pub;
    ros::Publisher load_cmd_pub;

    std::map<int, struct LocationSourceParam> location_source_params;
    prometheus_msgs::UAVState uav_state;
};

class LoadStartScript
{
public:
    LoadStartScript(ros::NodeHandle& nh)
    {
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<int>("flag", flag, 1);
        //【话题】订阅启动命令
        load_cmd_sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/prometheus/load_cmd", 10, &LoadStartScript::loadCmdCb, this);
        //【话题】发布消息反馈
        text_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(uav_id) + "/prometheus/text_info", 10);
    }
    
    std::string exec(const char* cmd) 
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        // 去除可能的空格
        result.erase(remove_if(result.begin(), result.end(), isspace), result.end());
        return result;
    }

    void loadCmdCb(const prometheus_msgs::StartScript::ConstPtr &msg)
    {
        std::string message;
        uint8_t message_type = prometheus_msgs::TextInfo::INFO;
        // 判断启动位置，是否正确
        if(msg->flag == flag){

            bool check = false;
            // 首先判断是否需要检测
            if(msg->mode == prometheus_msgs::StartScript::DETECTION_NODE_NAME){
                // msg->node_name 存储了节点名， 如果多个节点名则格式为 node1_name | node2_name 以此类推
                // 然后需要再当前的ROS中判断是否已经存在上述的节点
                check = checkNodesExist(msg->node_name);
            }else if(msg->mode == prometheus_msgs::StartScript::DETECTION_SHELL_CMD){
                check = (exec(msg->detection_cmd.c_str()) == "true");
            }

            // 通过检查已经启动了
            if(check){
                // 判断：10s内反复运行2次则进行关闭，反之消息反馈提示
                // 说明已经重复点击过了
                if(repeat_load_cmd_time.find(msg->cmd) != repeat_load_cmd_time.end()){
                    // 计算时间 如果小于10s
                    ros::Time now_time = ros::Time::now();
                    if((now_time - repeat_load_cmd_time[msg->cmd]).toSec() < 10.0 && (now_time - repeat_load_cmd_time[msg->cmd]).toSec() > 3.0){
                        // 运行关闭
                        system(msg->close_cmd.c_str());
                        message = "LoadStartScript: The program has been closed. Click again and restart.";
                        repeat_load_cmd_time.erase(repeat_load_cmd_time.find(msg->cmd));
                    }else{ // 如果大于10s，则重新记录时间
                        repeat_load_cmd_time[msg->cmd] = now_time;
                        message_type = prometheus_msgs::TextInfo::WARN;
                        message = "LoadStartScript: Click to close the program again within 10 seconds, the interval needs to exceed 3 seconds.";
                    }
                }else{ 
                    // 记录再次点击的第一次
                    repeat_load_cmd_time.insert({msg->cmd, ros::Time::now()});
                    message_type = prometheus_msgs::TextInfo::WARN;
                    message = "LoadStartScript: Click to close the program again within 10 seconds, the interval needs to exceed 3 seconds.";
                }
            }else
            {
                // 然后判断是否是定位驱动程序
                if(msg->cmd_level == prometheus_msgs::StartScript::LOCATION_SOURCE_CMD){
                    // 定位驱动程序一般来说只能存在一个
                    // 需要关闭之前的定位程序
                    if(!last_location_source_close.empty()){
                        system(last_location_source_close.c_str());
                    }
                    last_location_source_close = msg->close_cmd;
                }

                // 运行
                system(msg->cmd.c_str());
                message = "LoadStartScript: " + msg->cmd;
            }
            if(message_type == prometheus_msgs::TextInfo::INFO){
                ROS_INFO(message.c_str());
            }else if(message_type == prometheus_msgs::TextInfo::WARN){
                ROS_WARN(message.c_str());
            }else if(message_type == prometheus_msgs::TextInfo::ERROR || message_type == prometheus_msgs::TextInfo::FATAL){
                ROS_ERROR(message.c_str());
            }
            textInfoPub(message_type, message);
        }else{
            ROS_WARN("LoadStartScript: Machine startup error.");
        }
    }

    bool checkNodesExist(const std::string &node_names) 
    {
        // 解析节点名
        std::istringstream ss(node_names);
        std::string token;
        std::vector<std::string> node_list;

        while (std::getline(ss, token, '|')) {
            // 去除可能的空格
            token.erase(remove_if(token.begin(), token.end(), isspace), token.end());
            node_list.push_back(token);
        }

        // 获取当前活跃的节点
        ros::V_string v_nodes;
        ros::master::getNodes(v_nodes);
        if (!ros::master::getNodes(v_nodes)) {
            ROS_ERROR("Failed to get active nodes.");
            return false;
        }

        bool value = false;
        // 检查每个节点名是否存在
        for (const auto &node_name : node_list) {
            bool exists = false;
            for (const auto &node : v_nodes) {
                if (node_name == node) {
                    exists = true;
                    value = true;
                    break;
                }
            }
            if (exists) {
                ROS_INFO("Node %s exists.", node_name.c_str());
            } else {
                ROS_WARN("Node %s does not exist.", node_name.c_str());
            }
        }

        return value;
    }

    // 发布消息反馈
    void textInfoPub(int message_type, std::string message)
    {
        prometheus_msgs::TextInfo text_info;
        text_info.header.stamp = ros::Time::now();
        text_info.MessageType = message_type;
        text_info.Message = message;
        text_info_pub.publish(text_info);
    }

private:
    int uav_id;
    int flag;
    //【订阅话题】接收命令
    ros::Subscriber load_cmd_sub;
    //【发布话题】消息反馈
    ros::Publisher text_info_pub;

    // 记录启动的命令
    // std::map<std::string, struct CommandAttribute> command_attributes; 
    // 记录已经存在的重复点击命令的时间戳数据
    std::map<std::string, ros::Time> repeat_load_cmd_time;
    std::string last_location_source_close = "";
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "switch_location_source");
    ros::NodeHandle nh("~");
    bool is_switch_location_source, is_load_start_script;
    nh.param<bool>("switch_location_source_start_or_not", is_switch_location_source, true);
    nh.param<bool>("load_start_script_start_or_not", is_load_start_script, true);

    printf("\033[1;32m---->[switch_location_source] start running\n\033[0m");

    // 根据特定情况启动
    if(is_switch_location_source && is_load_start_script){
        SwitchLocationSource switch_location_source(nh);
        LoadStartScript load_start_script(nh);
        ros::spin();
    }else if(is_switch_location_source){
        SwitchLocationSource switch_location_source(nh);
        ros::spin();
    }else if(is_load_start_script){
        LoadStartScript load_start_script(nh);
        ros::spin();
    }

    return 0;
}
