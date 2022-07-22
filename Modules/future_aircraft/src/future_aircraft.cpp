/******************************************************************************
*例程简介: 
*
*效果说明: 
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/
#include <ros/ros.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include "printf_utils.h"

#include "mission_utils.h"

using namespace std;

//创建无人机相关数据变量
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;
Eigen::Matrix3f R_Body_to_ENU;      // 无人机机体系至惯性系转换矩阵
Detection_result ellipse_det;
float camera_offset[3];
//static target1
Eigen::Vector3d waypoint1 = {8.0, 0, 0};
Eigen::Vector3d waypoint2 = {-8.0, 2, 0};
Eigen::Vector3d waypoint3 = {8.0, 2, 0};
//static target2
Eigen::Vector3d waypoint4 = {0, 5, 2};//ENU
Eigen::Vector3d waypoint5 = {8.0, 0, 0};
Eigen::Vector3d waypoint6 = {-8.0, 2, 0};
Eigen::Vector3d waypoint7 = {8.0, 2, 0};
//move target3
Eigen::Vector3d waypoint8 = {10.0, 0, 2};//ENU
Eigen::Vector3d waypoint9 = {8.0, 2, 0};
Eigen::Vector3d waypoint10 = {-8.0, 2, 0};
Eigen::Vector3d waypoint11 = {8.0, 2, 0};
Eigen::Vector3d waypoint12 = {-8.0, 2, 0};
// 最大目标丢失计数
constexpr int max_loss_count = 30;
int loss_count = max_loss_count;

// 四种状态机
enum EXEC_STATE
{
    TAKEOFF,
    SEARCH,
    TRACKING,
    LOST,
    RETURN,
};
EXEC_STATE exec_state;

//无人机状态回调函数
void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    R_Body_to_ENU = get_rotation_matrix(uav_state.attitude[0],uav_state.attitude[1],uav_state.attitude[2]);

}

//无人机控制状态回调函数
void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

void ellipse_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    ellipse_det.object_name = "T";
    ellipse_det.Detection_info = *msg;
    ellipse_det.pos_body_frame[0] = -ellipse_det.Detection_info.position[1] + camera_offset[0];
    ellipse_det.pos_body_frame[1] = -ellipse_det.Detection_info.position[0] + camera_offset[1];
    ellipse_det.pos_body_frame[2] = -ellipse_det.Detection_info.position[2] + camera_offset[2];

    ellipse_det.pos_body_enu_frame = R_Body_to_ENU * ellipse_det.pos_body_frame;
    
    // 机体惯性系 -> 惯性系
    ellipse_det.pos_enu_frame[0] = uav_state.position[0] + ellipse_det.pos_body_enu_frame[0];
    ellipse_det.pos_enu_frame[1] = uav_state.position[1] + ellipse_det.pos_body_enu_frame[1];
    ellipse_det.pos_enu_frame[2] = uav_state.position[2] + ellipse_det.pos_body_enu_frame[2];
    // 此降落方案不考虑偏航角
    ellipse_det.att_enu_frame[2] = 0.0;

    if (ellipse_det.Detection_info.detected)
    {
        ellipse_det.num_regain++;
        ellipse_det.num_lost = 0;
    }
    else
    {
        ellipse_det.num_regain = 0;
        ellipse_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if (ellipse_det.num_lost > VISION_THRES)
    {
        ellipse_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if (ellipse_det.num_regain > VISION_THRES)
    {
        ellipse_det.is_detected = true;
    }
}

//主函数
int main(int argc, char** argv)
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "future_aircraft");
    //创建句柄
    ros::NodeHandle n;
    //声明起飞高度,无人机id变量
    float takeoff_height;
    int uav_id;
    //获取起飞高度参数
    ros::param::get("/uav_control_main_1/control/Takeoff_height", takeoff_height);
    ros::param::get("~uav_id", uav_id);
    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    n.param<float>("camera_offset_x", camera_offset[0], 0.0);
    n.param<float>("camera_offset_y", camera_offset[1], 0.0);
    n.param<float>("camera_offset_z", camera_offset[2], 0.0);
    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav1/prometheus/command", 10);
    //创建无人机状态命令订阅者
    ros::Subscriber uav_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav1/prometheus/state", 10, uav_state_cb);
    //创建无人机控制状态命令订阅者
    ros::Subscriber uav_control_state_sub = n.subscribe<prometheus_msgs::UAVControlState>("/uav1/prometheus/control_state", 10, uav_control_state_cb);

    ros::Subscriber ellipse_det_sub = n.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/ellipse_det", 10, ellipse_det_cb);
    //循环频率设置为1HZ
    ros::Rate r(1);
    //创建命令发布标志位,命令发布则为true;初始化为false
    bool cmd_pub_flag = false;
    uav_command.Command_ID = 1;
    exec_state = EXEC_STATE::TAKEOFF;

    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    //打印demo相关信息
    cout << GREEN << " [fly race demo " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Level: [Advance] " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Please use the RC SWA to armed, and the SWB to switch the drone to [COMMAND_CONTROL] mode  " << TAIL << endl;


    while(ros::ok())
    {
        //调用一次回调函数
        ros::spinOnce();
        // 等待进入COMMAND_CONTROL模式
        if (uav_control_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, TAIL, "Waiting for enter COMMAND_CONTROL state");
            continue;
        }
        switch (exec_state)
        {

            case TAKEOFF:
            {

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "ENU";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                if(fabs(uav_state.position[2] - takeoff_height) >= 0.2)
                {
                    continue;
                }
                cout << GREEN << " UAV takeoff successfully and search after 5 seconds" << TAIL << endl;
                exec_state = SEARCH;
            }
            case SEARCH:
            {
                if(ellipse_det.is_detected)
                {
                    exec_state = TRACKING;
                    loss_count = max_loss_count;
                }
                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                uav_command.position_ref[0] = waypoint1[0];
                uav_command.position_ref[1] = waypoint1[1];
                uav_command.position_ref[2] = waypoint1[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint1" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint2[0];
                uav_command.position_ref[1] = waypoint2[1];
                uav_command.position_ref[2] = waypoint2[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint2" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint3[0];
                uav_command.position_ref[1] = waypoint3[1];
                uav_command.position_ref[2] = waypoint3[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint3" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "ENU";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint4[0];
                uav_command.position_ref[1] = waypoint4[1];
                uav_command.position_ref[2] = waypoint4[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint4" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint5[0];
                uav_command.position_ref[1] = waypoint5[1];
                uav_command.position_ref[2] = waypoint5[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint5" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint6[0];
                uav_command.position_ref[1] = waypoint6[1];
                uav_command.position_ref[2] = waypoint6[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint6" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint7[0];
                uav_command.position_ref[1] = waypoint7[1];
                uav_command.position_ref[2] = waypoint7[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint7" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "ENU";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint8[0];
                uav_command.position_ref[1] = waypoint8[1];
                uav_command.position_ref[2] = waypoint8[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint8" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint9[0];
                uav_command.position_ref[1] = waypoint9[1];
                uav_command.position_ref[2] = waypoint9[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint9" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint10[0];
                uav_command.position_ref[1] = waypoint10[1];
                uav_command.position_ref[2] = waypoint10[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint10" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint11[0];
                uav_command.position_ref[1] = waypoint11[1];
                uav_command.position_ref[2] = waypoint11[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint11" << TAIL << endl;

                sleep(15);

                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "BODY";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS_BODY;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = waypoint12[0];
                uav_command.position_ref[1] = waypoint12[1];
                uav_command.position_ref[2] = waypoint12[2];
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "waypoint12" << TAIL << endl;

                sleep(30);

                exec_state = RETURN;


                
            }
            case TRACKING:
            {
                if (!ellipse_det.is_detected)
                {
                    --loss_count;
                    if(loss_count < 0)
                        exec_state = SEARCH;
                    break;
                }
                //坐标系
                uav_command.header.frame_id = "BODY";
                // Move模式
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                // 机体系下的速度控制
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS_BODY;
                // 使用机体惯性系作为误差进行惯性系的速度控制
                for (int i = 0; i < 3; i++)
                {
                    uav_command.velocity_ref[i] = 0.5 * ellipse_det.pos_body_enu_frame[i];
                }
                uav_command.position_ref[2] = 1.2;
                // 移动过程中，不调节航向角
                uav_command.yaw_ref = 0.0;
                // info << "Find object,Go to the target point > velocity_x: " << uav_command.velocity_ref[0] << " [m/s] "
                //     << "velocity_y: " << uav_command.velocity_ref[1] << " [m/s] "
                //     << std::endl;
                // PCOUT(1, GREEN, info.str());
                // if (std::abs(uav_command.velocity_ref[0]) + std::abs(uav_command.velocity_ref[1]) < 0.04)
                //     exec_state = LAND;
                break;
            }
            case RETURN:
            {
                // return to home 
                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "ENU";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                // uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
                uav_command.position_ref[0] = 0;
                uav_command.position_ref[1] = 0;
                uav_command.position_ref[2] = 1;
                uav_command.yaw_ref = 0.0;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "return to home" << TAIL << endl;

                sleep(15);
                uav_command.header.stamp = ros::Time::now();
                uav_command.header.frame_id = "ENU";
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                uav_command.Command_ID += 1;
                uav_command_pub.publish(uav_command);
                cout << GREEN << "landing" << TAIL << endl;
            }
        }
    }
    return 0;
}
