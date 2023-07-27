#include "ugv_controller.h"

UGV_controller::UGV_controller(ros::NodeHandle &nh)
{
    nh.param("ugv_id", this->ugv_id, 0);
    nh.param("swarm_num_ugv", this->swarm_num_ugv, 3);
    // 控制变量
    nh.param("k_p", this->k_p, 2.0f);
    nh.param("k_p_path", this->k_p_path, 5.0f);
    nh.param("k_aoivd", this->k_aoivd, 0.2f); 
    nh.param("k_yaw", this->k_yaw, 2.0f);
    nh.param("max_vel", this->max_vel, 2.0f);
    // 是否打印消息
    nh.param("flag_printf", this->flag_printf, false);
    // 地理围栏
    nh.param("geo_fence/x_min", this->geo_fence_x[0], -100.0f);
    nh.param("geo_fence/x_max", this->geo_fence_x[1], 100.0f);
    nh.param("geo_fence/y_min", this->geo_fence_y[0], -100.0f);
    nh.param("geo_fence/y_max", this->geo_fence_y[1], 100.0f);

    this->ugv_name = "/ugv" + std::to_string(this->ugv_id);

     //【订阅】无人车控制指令
    this->command_sub = nh.subscribe<prometheus_msgs::UgvCommand>(this->ugv_name + "/prometheus/ugv_command", 2, &UGV_controller::ugv_command_cb, this);
    //【订阅】本机状态信息
    this->ugv_state_sub = nh.subscribe<prometheus_msgs::UgvState>(this->ugv_name + "/prometheus/ugv_state", 2, &UGV_controller::ugv_state_cb, this);
    
    // 【订阅】所有状态信息（来自其他无人机 -> 通信节点）
    this->all_ugv_state_sub_ = nh.subscribe<prometheus_msgs::MultiUGVState>("/prometheus/all_ugv_state",
                                                                            1,
                                                                            &UGV_controller::allUGVStateCb, this);
    
    //【发布】底层控制指令
    this->cmd_pub = nh.advertise<geometry_msgs::Twist>(this->ugv_name + "/cmd_vel", 10);

    // 【定时器】
    ros::Timer debug_pub = nh.createTimer(ros::Duration(5.0), &UGV_controller::printf_state, this);




    // 初始化命令
    this->Command_Now.Mode                = prometheus_msgs::UgvCommand::Hold;
    this->Command_Now.Command_ID          = 0;
    this->Command_Now.linear_vel[0]       = 0;
    this->Command_Now.linear_vel[1]       = 0;
    this->Command_Now.angular_vel         = 0;

    this->vel_avoid_nei<<0,0;

    this->test_time = 0.0;

    this->only_rotate = true;
    this->error_yaw = 0.0;
    cout << GREEN << "ugv_controller_ugv_" <<  this->ugv_id << " init."<< TAIL << TAIL <<endl;  
    cout << GREEN << "ugv_name   : "<< this->ugv_name << TAIL <<endl; 
    cout << GREEN << "k_p_path    : "<< this->k_p_path <<"  "<< TAIL <<endl; 
    cout << GREEN << "k_aoivd    : "<< this->k_aoivd <<"  "<< TAIL <<endl;
    cout << GREEN << "k_yaw    : "<< this->k_yaw <<"  "<< TAIL <<endl; 
    cout << GREEN << "max_vel    : "<< this->max_vel <<"  "<< TAIL <<endl; 
    cout << GREEN << "geo_fence_x : "<< this->geo_fence_x[0] << " [m]  to  "<< this->geo_fence_x[1] << " [m]"<< TAIL << endl;
    cout << GREEN << "geo_fence_y : "<< this->geo_fence_y[0] << " [m]  to  "<< this->geo_fence_y[1] << " [m]"<< TAIL << endl;
}

void UGV_controller::mainloop()
{
   

        // Check for geo fence: If ugv is out of the geo fence, it will hold now.
        if(check_failsafe() == 1)  //out of the border
        {
            this->Command_Now.Mode = prometheus_msgs::UgvCommand::Hold;  //stop
        }

        switch (this->Command_Now.Mode)
        {
        // 【Start】 
        case prometheus_msgs::UgvCommand::Hold:
            
            this->cmd_vel.linear.x = 0.0;
            this->cmd_vel.linear.y = 0.0;
            this->cmd_vel.linear.z = 0.0;
            this->cmd_vel.angular.x = 0.0;
            this->cmd_vel.angular.y = 0.0;
            this->cmd_vel.angular.z = 0.0;
            this->cmd_pub.publish(this->cmd_vel);
            break;

        case prometheus_msgs::UgvCommand::Direct_Control_BODY:  //speed control by vx,vy

            // 注: linear.x与linear.y控制的是无人车车体系下的线速度
            this->cmd_vel.linear.x = this->Command_Now.linear_vel[0];
            this->cmd_vel.linear.y = this->Command_Now.linear_vel[1];
            this->cmd_vel.linear.z = 0.0;
            this->cmd_vel.angular.x = 0.0;
            this->cmd_vel.angular.y = 0.0;
            this->cmd_vel.angular.z = this->Command_Now.angular_vel;
            this->cmd_pub.publish(this->cmd_vel);
            break;

        case prometheus_msgs::UgvCommand::Direct_Control_ENU:  //speed control of yaw angle
            
            this->error_yaw = this->Command_Now.yaw_ref- this->yaw_ugv;

            if(this->error_yaw < -M_PI)
            {
                this->error_yaw = this->error_yaw + 2*M_PI;
            }else if(this->error_yaw > M_PI)
            {
                this->error_yaw = this->error_yaw - 2*M_PI;
            }

            if( abs(this->error_yaw) < 5.0/180.0 * M_PI)  //small angle: direct movement
            {
                float body_x, body_y;

                body_x = this->Command_Now.linear_vel[0] * cos(this->yaw_ugv) + this->Command_Now.linear_vel[1] * sin(this->yaw_ugv);
                body_y = -this->Command_Now.linear_vel[0] * sin(this->yaw_ugv) + this->Command_Now.linear_vel[1] * cos(this->yaw_ugv);

                this->cmd_vel.linear.x = body_x;
                this->cmd_vel.linear.y = body_y;
                this->cmd_vel.linear.z = 0.0;
                this->cmd_vel.angular.x = 0.0;
                this->cmd_vel.angular.y = 0.0;
                this->cmd_vel.angular.z = this->k_yaw * this->error_yaw;
            }else                                                                  //large angle: same place adjustment 
            {
                // 先调整yaw
                this->cmd_vel.linear.x = 0.0;
                this->cmd_vel.linear.y = 0.0;
                this->cmd_vel.linear.z = 0.0;
                this->cmd_vel.angular.x = 0.0;
                this->cmd_vel.angular.y = 0.0;
                this->cmd_vel.angular.z = this->k_yaw*this->error_yaw;
            }

            this->cmd_pub.publish(this->cmd_vel);
            break;

        case prometheus_msgs::UgvCommand::Point_Control:
            
            // Command_Now.yaw_ref = (-180,180]
            // yaw_ugv = (-180,180] not sure
            // error_yaw = (-180,180] 

            this->error_yaw = this->Command_Now.yaw_ref - this->yaw_ugv;

            if(this->error_yaw < -M_PI)
            {
                this->error_yaw = this->error_yaw + 2*M_PI;
            }else if(this->error_yaw > M_PI)
            {
                this->error_yaw = this->error_yaw - 2*M_PI;
            }

            if( abs(this->error_yaw) < 5.0/180.0 * M_PI)
            {
                float enu_x,enu_y;
                enu_x = this->k_p*(this->Command_Now.position_ref[0] - this->pos_ugv[0]);
                enu_y = this->k_p*(this->Command_Now.position_ref[1] - this->pos_ugv[1]);
                float body_x, body_y;
                body_x = enu_x * cos(this->yaw_ugv) + enu_y * sin(this->yaw_ugv);
                body_y = -enu_x * sin(this->yaw_ugv) + enu_y * cos(this->yaw_ugv);

                this->cmd_vel.linear.x = body_x;
                this->cmd_vel.linear.y = body_y;
                this->cmd_vel.linear.z = 0.0;
                this->cmd_vel.angular.x = 0.0;
                this->cmd_vel.angular.y = 0.0;
                this->cmd_vel.angular.z = this->k_yaw*this->error_yaw;
            }else
            {
                // 先调整yaw
                this->cmd_vel.linear.x = 0.0;
                this->cmd_vel.linear.y = 0.0;
                this->cmd_vel.linear.z = 0.0;
                this->cmd_vel.angular.x = 0.0;
                this->cmd_vel.angular.y = 0.0;
                this->cmd_vel.angular.z = this->k_yaw*this->error_yaw;
            }

            // 速度限制幅度 
            if(this->cmd_vel.linear.x > this->max_vel)
            {
                this->cmd_vel.linear.x = this->max_vel;
            }else if(this->cmd_vel.linear.x < -this->max_vel)
            {
                this->cmd_vel.linear.x = -this->max_vel;
            }

            if(this->cmd_vel.linear.y > this->max_vel)
            {
                this->cmd_vel.linear.y = this->max_vel;
            }else if(this->cmd_vel.linear.y < -this->max_vel)
            {
                this->cmd_vel.linear.y = -this->max_vel;
            }
            this->cmd_pub.publish(this->cmd_vel);
            break;

        case prometheus_msgs::UgvCommand::Path_Control:  //more vel_avoid_nei than point control

            this->error_yaw = this->Command_Now.yaw_ref- this->yaw_ugv;

            if(this->error_yaw < -M_PI)
            {
                this->error_yaw = this->error_yaw + 2*M_PI;
            }else if(this->error_yaw > M_PI)
            {
                this->error_yaw = this->error_yaw - 2*M_PI;
            }

            if( abs(this->error_yaw) < 5.0/180.0 * M_PI)
            {
                float enu_x,enu_y;
                enu_x = this->k_p_path*(this->Command_Now.position_ref[0] - this->pos_ugv[0]);
                enu_y = this->k_p_path*(this->Command_Now.position_ref[1] - this->pos_ugv[1]);
                // cal vel_avoid_nei
                add_apf_vel();
                enu_x = enu_x + this->vel_avoid_nei[0];
                enu_y = enu_y + this->vel_avoid_nei[1];
                float body_x, body_y;
                body_x = enu_x * cos(this->yaw_ugv) + enu_y * sin(this->yaw_ugv);
                body_y = -enu_x * sin(this->yaw_ugv) + enu_y * cos(this->yaw_ugv);

                this->cmd_vel.linear.x = body_x;
                this->cmd_vel.linear.y = body_y;
                this->cmd_vel.linear.z = 0.0;
                this->cmd_vel.angular.x = 0.0;
                this->cmd_vel.angular.y = 0.0;
                this->cmd_vel.angular.z = this->k_yaw*this->error_yaw;
            }else
            {
                // 先调整yaw
                this->cmd_vel.linear.x = 0.0;
                this->cmd_vel.linear.y = 0.0;
                this->cmd_vel.linear.z = 0.0;
                this->cmd_vel.angular.x = 0.0;
                this->cmd_vel.angular.y = 0.0;
                this->cmd_vel.angular.z = this->k_yaw * this->error_yaw;
            }

            // 速度限制幅度 
            if(this->cmd_vel.linear.x > this->max_vel)
            {
                this->cmd_vel.linear.x = this->max_vel;
            }else if(this->cmd_vel.linear.x < -this->max_vel)
            {
                this->cmd_vel.linear.x = -this->max_vel;
            }

            if(this->cmd_vel.linear.y > this->max_vel)
            {
                this->cmd_vel.linear.y = this->max_vel;
            }else if(this->cmd_vel.linear.y < -this->max_vel)
            {
                this->cmd_vel.linear.y = -this->max_vel;
            }
            this->cmd_pub.publish(this->cmd_vel);
            break;

        case prometheus_msgs::UgvCommand::Test:  //测试程序：走圆
            
            Circle_trajectory_generation(test_time);

            test_time = test_time + 0.05;       //20Hz
            break;

        }

        this->Command_Last = this->Command_Now;
    
}


void UGV_controller::ugv_command_cb(const prometheus_msgs::UgvCommand::ConstPtr& msg)
{
    this->Command_Now = *msg;
    this->only_rotate = true; // vinson: should be set for initializing.
    this->first_arrive = true;
    this->stop_flag = false;

    // Command_Now.yaw_ref = (-180,180]
    if(this->Command_Now.yaw_ref > M_PI + 0.001f)
    {
        this->Command_Now.yaw_ref = 0.0;
        cout << RED << "Wrong Command_Now.yaw_ref : " <<  this->Command_Now.yaw_ref << " , Reset to 0." << TAIL <<endl; 
    }else if(this->Command_Now.yaw_ref < -M_PI - 0.001f)
    {
        this->Command_Now.yaw_ref = 0.0;
        cout << RED << "Wrong Command_Now.yaw_ref : " <<  this->Command_Now.yaw_ref << " , Reset to 0." << TAIL <<endl; 
    }else if(this->Command_Now.yaw_ref == -M_PI)
    {
        this->Command_Now.yaw_ref = M_PI;
    }
}

void UGV_controller::ugv_state_cb(const prometheus_msgs::UgvState::ConstPtr& msg)
{
    this->_UgvState = *msg;
    this->pos_ugv  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    this->vel_ugv  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    this->yaw_ugv = this->_UgvState.attitude[2];

    if(this->flag_printf)
    {
        float vel_ugv = sqrtf((pow(this->_UgvState.velocity[0],2) + pow(this->_UgvState.velocity[1],2)));
        // if(vel_ugv > 1.0)
        // {
        //     cout << RED << ugv_name + " /velocity more than 1 m/s: [ " << vel_ugv <<  " ] "<< TAIL<<endl;
        // }
    }
}

void UGV_controller::allUGVStateCb(const prometheus_msgs::MultiUGVState::ConstPtr &msg)
{
    this->swarm_num_ugv = msg->swarm_num_ugv;
    id = this->ugv_id - 1;
    for(int i = 0; i < this->swarm_num_ugv; i++)
    {
        this->all_ugv_status_[i] = msg->ugv_state_all[i];
        this->all_ugv_pos_[i] = Eigen::Vector3d(msg->ugv_state_all[i].position[0], 
                                    msg->ugv_state_all[i].position[1], msg->ugv_state_all[i].position[2]);
        this->all_ugv_vel_[i] = Eigen::Vector3d(msg->ugv_state_all[i].velocity[0], 
                                    msg->ugv_state_all[i].velocity[1], msg->ugv_state_all[i].velocity[2]);
    }
}

void UGV_controller::add_apf_vel()
{
    this->vel_avoid_nei << 0.0,0.0;
    float R = 1.2;
    float r = 0.5;

    // 增加无人车之间的局部躲避
    for(int i = 1; i <= this->swarm_num_ugv; i++)
    {
        if(i == this->ugv_id)
        {
            continue;
        }

        float distance_to_nei = (pos_ugv - this->all_ugv_vel_[i-1]).norm();
        if(distance_to_nei  > R )
        {
            continue;
        }else if(distance_to_nei  > r)
        {
            this->vel_avoid_nei[0] = this->vel_avoid_nei[0] +  k_aoivd * (this->pos_ugv[0] - this->all_ugv_vel_[i-1][0]);
            this->vel_avoid_nei[1] = this->vel_avoid_nei[1] +  k_aoivd * (this->pos_ugv[1] - this->all_ugv_vel_[i-1][1]);
        }
    }
}

void UGV_controller::Circle_trajectory_generation(float time_from_start)
{
    float omega;
    float linear_vel = 0.3;
    float circle_radius = 1.0;

    omega = fabs(linear_vel / circle_radius);

    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    this->Command_Now.position_ref[0] = circle_radius * cos_angle + 0.5;
    this->Command_Now.position_ref[1] = circle_radius * sin_angle + 0.0;
    this->Command_Now.yaw_ref = 0.0;

    this->error_yaw = this->Command_Now.yaw_ref- this->yaw_ugv;

    if(this->error_yaw < -M_PI)
    {
        this->error_yaw = this->error_yaw + 2*M_PI;
    }else if(this->error_yaw > M_PI)
    {
        this->error_yaw = this->error_yaw - 2*M_PI;
    }

    if( abs(this->error_yaw) < 5.0/180.0 * M_PI)
    {
        float enu_x,enu_y;
        enu_x = this->k_p*(this->Command_Now.position_ref[0] - this->pos_ugv[0]);
        enu_y = this->k_p*(this->Command_Now.position_ref[1] - this->pos_ugv[1]);
        float body_x, body_y;
        body_x = enu_x * cos(this->yaw_ugv) + enu_y * sin(this->yaw_ugv);
        body_y = -enu_x * sin(this->yaw_ugv) + enu_y * cos(this->yaw_ugv);

        this->cmd_vel.linear.x = body_x;
        this->cmd_vel.linear.y = body_y;
        this->cmd_vel.linear.z = 0.0;
        this->cmd_vel.angular.x = 0.0;
        this->cmd_vel.angular.y = 0.0;
        this->cmd_vel.angular.z = this->k_yaw*this->error_yaw;
    }else
    {
        // 先调整yaw
        this->cmd_vel.linear.x = 0.0;
        this->cmd_vel.linear.y = 0.0;
        this->cmd_vel.linear.z = 0.0;
        this->cmd_vel.angular.x = 0.0;
        this->cmd_vel.angular.y = 0.0;
        this->cmd_vel.angular.z = this->k_yaw*this->error_yaw;
    }

    // 速度限制幅度 
    if(this->cmd_vel.linear.x > this->max_vel)
    {
        this->cmd_vel.linear.x = this->max_vel;
    }else if(this->cmd_vel.linear.x < -this->max_vel)
    {
        this->cmd_vel.linear.x = -this->max_vel;
    }

    if(this->cmd_vel.linear.y > this->max_vel)
    {
        this->cmd_vel.linear.y = this->max_vel;
    }else if(this->cmd_vel.linear.y < -this->max_vel)
    {
        this->cmd_vel.linear.y = -this->max_vel;
    }
    this->cmd_pub.publish(this->cmd_vel);
}


void UGV_controller::printf_state(const ros::TimerEvent &e)
{
    if(!this->flag_printf)
    {
        return;
    }

    cout << GREEN <<">>>>>>>>>>>>>>>>>>>>>>UGV[" << ugv_id <<  "] State" << "<<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl; 
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << GREEN << "UAV_name : " <<  this->ugv_name << endl;
    cout << GREEN << "UAV_pos [X Y Z] : " << this->pos_ugv[0] << " [ m ] "<< this->pos_ugv[1]<<" [ m ] "<< this->pos_ugv[2]<<" [ m ] "<< TAIL <<endl; 
    cout << GREEN << "UAV_vel [X Y Z] : " << this->vel_ugv[0] << " [ m/s ] "<< this->vel_ugv[1]<<" [ m/s ] "<< this->vel_ugv[2]<<" [ m/s ] "<< TAIL <<endl; 
    cout << GREEN << "Yaw             : " << this->yaw_ugv * 180/M_PI<<" [deg] "<< TAIL <<endl; 
    cout << GREEN << "Battery         : " << this->_UgvState.battery<<" [V] "<< TAIL <<endl; 

    switch(this->Command_Now.Mode)
    {
        case prometheus_msgs::UgvCommand::Hold:
            cout << GREEN << "Command: [ Hold ] " << TAIL <<endl; 
            break;

        case prometheus_msgs::UgvCommand::Direct_Control_BODY:
            cout << GREEN << "Command: [ Direct_Control_BODY ] " << TAIL <<endl; 
            break;

        case prometheus_msgs::UgvCommand::Direct_Control_ENU:
            cout << GREEN << "Command: [ Direct_Control_ENU ] " << TAIL <<endl; 

            break;

        case prometheus_msgs::UgvCommand::Point_Control:
            cout << GREEN << "Command: [ Point_Control ] " << TAIL <<endl; 
            cout << GREEN << "Pos_ref [X Y] : " << this->Command_Now.position_ref[0]  << " [ m ] "<< this->Command_Now.position_ref[1] <<" [ m ] "<< TAIL <<endl; 
            break;

        case prometheus_msgs::UgvCommand::Path_Control:
            cout << GREEN << "Command: [ Path_Control ] " << TAIL <<endl; 
            break;

        case prometheus_msgs::UgvCommand::Test:
            cout << GREEN << "Command: [ Test ] " << TAIL <<endl; 
            break;
    }

}
//ego boundary of ugv 
int UGV_controller::check_failsafe()
{
    if (this->_UgvState.position[0] < this->geo_fence_x[0] || this->_UgvState.position[0] > this->geo_fence_x[1] ||
        this->_UgvState.position[1] < this->geo_fence_y[0] || this->_UgvState.position[1] > this->geo_fence_y[1])
    {
        cout << RED << "Out of the geo fence, stop!" << TAIL <<endl;  
        return 1;
    }
    else
    {
        return 0;
    }
}
