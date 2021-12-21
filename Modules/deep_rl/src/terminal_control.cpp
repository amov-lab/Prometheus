#include <ros/ros.h>
#include <iostream>

#include <prometheus_drl/ugv_move_cmd.h>
#include <prometheus_drl/ugv_reset.h>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh("~");

    //　【发布】　控制指令
    ros::Publisher move_pub = nh.advertise<prometheus_drl::ugv_move_cmd>("/ugv1/move_cmd", 10);
    //　【发布】　reset指令
    ros::Publisher reset_pub = nh.advertise<prometheus_drl::ugv_reset>("/reset", 10);

    prometheus_drl::ugv_move_cmd move_cmd;
    prometheus_drl::ugv_reset reset;

    move_cmd.ID = 0;
    move_cmd.CMD = prometheus_drl::ugv_move_cmd::HOLD;

    int cmd_mode;

    while(ros::ok())
    {
        cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
        cout << "Please select cmd: 0 for hold, 1 for forward, 2 for back, 3 for left, 4 for right, 99 for reset.."<<endl;
        cin >> cmd_mode;

        if(cmd_mode == 0)
        {
            move_cmd.ID += 1;
            move_cmd.CMD = prometheus_drl::ugv_move_cmd::HOLD;
            move_pub.publish(move_cmd);
        }else if(cmd_mode == 1)
        {
            move_cmd.ID += 1;
            move_cmd.CMD = prometheus_drl::ugv_move_cmd::FORWARD;
            move_pub.publish(move_cmd);
        }else if(cmd_mode == 2)
        {
            move_cmd.ID += 1;
            move_cmd.CMD = prometheus_drl::ugv_move_cmd::BACK;
            move_pub.publish(move_cmd);
        }else if(cmd_mode == 3)
        {
            move_cmd.ID += 1;
            move_cmd.CMD = prometheus_drl::ugv_move_cmd::LEFT;
            move_pub.publish(move_cmd);
        }else if(cmd_mode == 4)
        {
            move_cmd.ID += 1;
            move_cmd.CMD = prometheus_drl::ugv_move_cmd::RIGHT;
            move_pub.publish(move_cmd);
        }else if(cmd_mode == 99)
        {
            move_cmd.ID = 0;
            reset.reset = 1;
            reset_pub.publish(reset);
        }else
        {
            ROS_ERROR("wrong input");
        }
        

        sleep(0.5);
    }

    return 0;
}
