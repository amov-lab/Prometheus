#include <ros/ros.h>
#include <prometheus_msgs/Msg103.h>
#include <prometheus_msgs/Msg104.h>

using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "cxy_msg_103_104");
    ros::NodeHandle nh("~");
            
    ros::Publisher msg103_pub = nh.advertise<prometheus_msgs::Msg103>("/kongdijiqun/msg103", 1); 
    ros::Publisher msg104_pub = nh.advertise<prometheus_msgs::Msg104>("/kongdijiqun/msg104", 1); 

    prometheus_msgs::Msg103 msg_103;
    prometheus_msgs::Msg104 msg_104;
 
    int start_flag = 0;
    while(start_flag == 0 && ros::ok())
    {
        cout << "1 for msg_103, 2 for msg_104."<<endl;
        cin >> start_flag;

        if(start_flag == 1)
        {
            cout << "msg_103."<<endl;
        }else
        {
            cout << "msg_104."<<endl;
        }
        
        sleep(0.2);
    }
    
    int command;
    float state_desired[2];
    while(ros::ok())
    {
        if(start_flag == 1)
        {
            cout << "Please choose command: 1 for uav start, 2 for return, 3 for stop, 4 for ugv start."<<endl;
            cin >> command;

            if(command==1)
            {
                msg_103.command = 1;
                msg103_pub.publish(msg_103);
                cout << "UAV Start."<<endl;
            }else if(command==2)
            {
                msg_103.command = 2;
                msg103_pub.publish(msg_103);
                cout << "Return."<<endl;
            }else if(command==3)
            {
                msg_103.command = 3;
                msg103_pub.publish(msg_103);
                cout << "Stop."<<endl;
            }else if(command==4)
            {
                cout << "Please enter ugv goal: "<<endl;
                cout << "desired state: --- x [m] "<<endl;
                cin >> state_desired[0];
                cout << "desired state: --- y [m]"<<endl;
                cin >> state_desired[1];

                msg_103.command = 4;
                msg_103.enu_position[0] = state_desired[0];
                msg_103.enu_position[1] = state_desired[1];
                msg_103.enu_position[2] = 0.0;
                msg103_pub.publish(msg_103);
                cout << "UGV Start."<<endl;
            }else
            {
                cout << "Wrong input..."<<endl;
            }
        }else if(start_flag == 2)
        {
            cout << "Please choose command: 1 for start, 2 for return, 3 for stop."<<endl;
            cin >> command;

            if(command==1)
            {
                msg_104.command = 1;
                msg104_pub.publish(msg_104);
                cout << "UAV Start."<<endl;
            }else if(command==2)
            {
                msg_104.command = 2;
                msg104_pub.publish(msg_104);
                cout << "Stop."<<endl;
            }else if(command==3)
            {
                msg_104.command = 3;
                msg104_pub.publish(msg_104);
                cout << "Stop."<<endl;
            }else
            {
                cout << "Wrong input..."<<endl;
            }
        }

        sleep(0.5);
    }

    return 0;
}
