#### uav_control模块逻辑

**遥控器4个拨杆**

- 拨杆1：进入程序控制（HOVER_CONTROL）
- 拨杆2：进入指令控制（COMMAND_CONTROL）
- 拨杆3：原地降落（LAND_CONTROL）
- 拨杆4：重启飞控（只有在MANUAL_CONTROL且未解锁时）

**设想的流程**

- 解锁，默认为MANUAL_CONTROL模式，此时可以正常控制无人机进行手动控制
- 在MANUAL_CONTROL模式中，可以通过手动控制确定PX4姿态环没有问题
- 在MANUAL_CONTROL模式中轻轻飞起飞机，然后使用遥控器拨杆1切换进入HOVER_CONTROL模式，此时无人机会自动悬停
- 在HOVER_CONTROL模式中，可以通过摇杆进行控制前后左右上下，类似PX4原生的定点模式
- 在HOVER_CONTROL模式中，可以用于确认里程计信息是否稳定准确，如果出现问题，程序会自动降落，但也可以手动切换回MANUAL_CONTROL进行降落
- 在HOVER_CONTROL模式中，可以遥控器拨杆2切换进入COMMAND_CONTROL模式，此时所有期望信息来自于程序
- 在COMMAND_CONTROL模式中，可以进行多种控制

- 在HOVER_CONTROL或者COMMAND_CONTROL中，都可以使用拨杆切换至LAND_CONTROL进行降落

- 无人机利用LAND_CONTROL将落后，此时程序自动切换为MANUAL_CONTROL模式，可以将所有拨杆归零，继续进行操控

**集群控制**

- 地面站（发布：起飞、阵型切换、降落、紧急降落等顶层指令） -> 通信节点（转发为ROS MSG）  -> 集群控制节点（根据当前状态及地面站指令发送/prometheus/command）-> uav_control节点
- 集群控制节点中订阅无人机状态信息 - "/uav"+std::to_string(uav_id)+"/prometheus/state"
- 集群控制节点中发布指令控制信息 - "/uav"+std::to_string(uav_id)+ "/prometheus/command"
- 对于集群位置控制：
  - 先发送Init_Pos_Hover指令，即起飞
  - 编队控制发送Move指令，子模式选择XYZ_POS，直接指定每个无人机的位置
- 对于集群速度控制：
  - 先发送Init_Pos_Hover指令，即起飞
  - 编队控制发送Move指令，子模式选择XY_VEL_Z_POS
- 对于集群加速度控制：
  - 先发送Init_Pos_Hover指令，即起飞
  - 编队控制发送Move指令，子模式选择XYZ_ATT

**对于MATLAB控制**

- 需要将controller_flag设定为CONTOLLER_FLAG::EXTERNAL_CONTROLLER
- MATLAB中订阅无人机状态信息 - "/uav"+std::to_string(uav_id)+"/prometheus/state"
- MATLAB中发布指令控制信息 - "/uav"+std::to_string(uav_id)+ "/prometheus/command"
- 按照正常流程手动起飞无人机，然后切换至HOVER_CONTROL，此时使用MATLAB发布控制指令，然后切换至COMMAND_CONTROL（移动子模式使用XYZ_ATT）
- 此时Promehteus将使用来自MATLAB的指令

关于controller_flag的说明：

- PX4_ORIGIN指使用PX4中的原生控制器，类似之前的px4_sender
  - 但是当控制指令的子模式设置为XYZ_ATT时，属于外部控制情况，如matlab
- PID，UDE，NE属于三种Prometheus中自带的三种位置环控制器，仅位置定点或者轨迹控制，不支持其他复合模式

主控制程序状态

- MANUAL_CONTROL             // 手动定点控制
  - 默认状态，即手动自稳模式
  - 遥控器拨杆1切换进入HOVER_CONTROL模式
    - 如果没有odom失效（失效判定权在estimator），拒绝进入
    - 如果收到CMD信息，拒绝进入
    - 设定当前odom点为悬停点，并将PX4切换进入OFFBOARD模式
  - 遥控器拨杆3切换重新启动飞控
    - 必须是上锁状态
- HOVER_CONTROL                // 悬停状态
  - 自动悬停模式，可以通过摇杆进行控制前后左右上下，类似PX4原生的定点模式
  - 遥控器拨杆2切换进入COMMAND_CONTROL模式
  - 如果此时切换拨杆1，无人机会直接降落
  - 由于担心用户手动操作水平，没有设置逻辑切换回MANUAL_CONTROL
- COMMAND_CONTROL         // 指令控制（必须先进入HOVER_CONTROL状态）
  - 指令控制模式，此时所有期望信息来自于程序
  - 遥控器拨杆2切换进入HOVER_CONTROL 模式
- LAND_CONTROL                   // 降落（必须在HOVER_CONTROL或者COMMAND_CONTROL中）
  - 自动降落，有两种降落情况，正常降落和快速降落
  - 快速降落一般用于紧急情况，降落速度较快
  - 降落至指定高度，无人机将自动上锁，且自动切换回MANUAL_CONTROL模式



#### PX4CtrlFSM逻辑

遥控器3个拨杆

- 拨杆1：api mode
- 拨杆2：cmd mode
- 拨杆3：重启飞控



控制模块状态

MANUAL_CTRL

- 即自稳模式
- 遥控器拨杆1切换进入AUTO_HOVER模式
  - 如果没有odom信息，拒绝进入
  - 如果收到CMD信息，拒绝进入
  - 如果odom的速度大于5米每秒，拒绝进入
  - 设定当前odom点为悬停点，并自动切换进入OFFBOARD模式
- 遥控器拨杆3切换重新启动飞控
  - 必须是上锁状态

AUTO_HOVER

- 自动根据遥控器指令和当前位置悬停
  - 如果刚进入则是当前点悬停
  - 如果是持续在该状态则是依据遥控器拨杆控制悬停
- 如果遥控器拨杆1切换或者odom失效，则进入MANUAL_CTRL模式
  - 关闭offboard模式
- 遥控器拨杆2切换，且收到cmd指令，进入CMD_CTRL模式

CMD_CTRL

- 遥控器拨杆1切换或者odom失效，则进入MANUAL_CTRL模式
- 遥控器拨杆2切换，或者没有收到cmd指令，进入AUTO_HOVER模式

无人机控制模式