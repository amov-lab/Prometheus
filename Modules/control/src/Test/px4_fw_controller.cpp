#include <ros/ros.h>

#include <state_from_mavros.h>
#include <command_to_mavros.h>

#include <pos_controller_PID.h>

#include <px4_command/ControlCommand.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command_utils.h>
#include <px4_command/Trajectory.h>

#include <Eigen/Eigen>

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
px4_command::ControlCommand Command_Now;                      //无人机当前执行命令
px4_command::ControlCommand Command_Last;                     //无人机上一条执行命令

px4_command::DroneState _DroneState;                          //无人机状态量
float cur_time;

Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
double yaw_sp;

//Target pos of the drone [from fcu]
Eigen::Vector3d pos_drone_fcu_target;
//Target vel of the drone [from fcu]
Eigen::Vector3d vel_drone_fcu_target;


void Command_cb(const px4_command::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}
void drone_state_cb(const px4_command::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    _DroneState.time_from_start = cur_time;
}
void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

    vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_fw_controller");
    ros::NodeHandle nh("~");

    ros::Subscriber Command_sub = nh.subscribe<px4_command::ControlCommand>("/px4_command/control_command", 10, Command_cb);

    //【订阅】无人机当前状态
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<px4_command::DroneState>("/px4_command/drone_state", 10, drone_state_cb);

    // 【订阅】无人机期望位置/速度/加速度 坐标系:ENU系
    //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为POSITION_TARGET_LOCAL_NED, 对应的飞控中的uORB消息为vehicle_local_position_setpoint.msg
    ros::Subscriber position_target_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, pos_target_cb);

    // 【发布】位置/速度/加速度期望值 坐标系 ENU系
    //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // 先读取一些飞控的数据
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode = command_to_mavros::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
    Command_Now.Reference_State.position_ref[0] = 0;
    Command_Now.Reference_State.position_ref[1] = 0;
    Command_Now.Reference_State.position_ref[2] = 0;
    Command_Now.Reference_State.velocity_ref[0] = 0;
    Command_Now.Reference_State.velocity_ref[1] = 0;
    Command_Now.Reference_State.velocity_ref[2] = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref = 0;


    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0;

    mavros_msgs::PositionTarget pos_setpoint;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
       // 当前时间
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        //执行回调函数
        ros::spinOnce();

        cout <<">>>>>>>>>>>>>>>>>>>>>> px4_fw_controller <<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        // 打印无人机状态
        px4_command_utils::prinft_drone_state(_DroneState);

        // 打印上层控制指令
        px4_command_utils::printf_command_control(Command_Now);


        switch (Command_Now.Mode)
        {
        case command_to_mavros::Idle:
            
            break;
            
        case command_to_mavros::Takeoff:

            break;

        // 不支持复合模式
        case command_to_mavros::Move_ENU:

            
            //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
            //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
            //Bit 10 should set to 0, means is not force sp
            pos_setpoint.type_mask = 0b110111000000;  // 110 111 000 000  vxvyvz + xyz

            pos_setpoint.coordinate_frame = 1;

            pos_setpoint.position.x = Command_Now.Reference_State.position_ref[0];
            pos_setpoint.position.y = Command_Now.Reference_State.position_ref[1];
            pos_setpoint.position.z = Command_Now.Reference_State.position_ref[2];



            pos_setpoint.velocity.x = 0.0;
            pos_setpoint.velocity.y = 0.0;
            pos_setpoint.velocity.z = Command_Now.Reference_State.yaw_ref /3.14 * 180;

            setpoint_raw_local_pub.publish(pos_setpoint);

            // 检查飞控是否收到控制量
            cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
            cout << "Pos_target [X Y Z] : " << pos_drone_fcu_target[0] << " [ m ] "<< pos_drone_fcu_target[1]<<" [ m ] "<<pos_drone_fcu_target[2]<<" [ m ] "<<endl;
            cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;

            break;

        // 不支持复合模式
        case command_to_mavros::Move_Body:


            break;

        case command_to_mavros::Hold:

            break;


        case command_to_mavros::Land:

            break;

        case command_to_mavros::Disarm:

            break;

        case command_to_mavros::PPN_land:

            break;

        }



        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;

}