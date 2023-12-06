#ifndef STRUCT_HPP
#define STRUCT_HPP

#include <vector>
#include <cstddef> // NULL
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

#define PROTOCOL_VERSION 1

enum MsgId
{
    UAVSTATE = 1,
    TEXTINFO = 3,
    GIMBALSTATE = 4,
    VISIONDIFF = 5,
    HEARTBEAT = 6,
    UGVSTATE = 7,
    MULTIDETECTIONINFO = 8,
    UAVCONTROLSTATE = 9,

    SWARMCOMMAND = 101,
    GIMBALCONTROL = 102,
    GIMBALSERVICE = 103,
    WINDOWPOSITION = 104,
    UGVCOMMAND = 105,
    GIMBALPARAMSET = 106,
    IMAGEDATA = 107,
    UAVCOMMAND = 108,
    UAVSETUP = 109,
    PARAMSETTINGS = 110,
    BSPLINE = 111,
    MULTIBSPLINES = 112,
    CUSTOMDATASEGMENT_1 = 113,

    CONNECTSTATE = 201,
    MODESELECTION = 202,

    //rviz数据
    UGVLASERSCAN = 230,
    UGVPOINTCLOUND2 = 231,

    UGVTFMESSAGE = 232,
    UGVTFSTATIC = 233,

    UGVMARKERARRAY = 234,
    UGVMARKERARRAYLANDMARK = 235,
    UGVMARKERARRAYTRAJECTORY = 236,

    GOAL = 255
};

//参考文件： UAVState.msg
//订阅话题： /uav*/prometheus/state
struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};
//MSG 1
struct UAVState
{
    //时间戳
    uint32_t secs;
    uint32_t nsecs;

    //无人机编号
    uint8_t uav_id;

    //无人机定位来源
    uint8_t location_source;

    //enum agent定位来源枚举
    enum LocationSource
    {
        MOCAP = 0,
        T265 = 1,
        GAZEBO = 2,
        FAKE_ODOM = 3,
        GPS = 4,
        RTK = 5,
        UWB = 6
    };

    // PX4飞控当前飞行模式  int8
    std::string mode;
    // 机载电脑是否连接上飞控，true已连接，false则不是
    bool connected;
    // 是否解锁，true为已解锁，false则不是
    bool armed;
    // odom失效
    bool odom_valid;
    // GPS状态,变量对应状态可参考mavros_msgs/GPSRAW中的fix_type
    uint8_t gps_status;
    // GPS状态枚举
    enum GPSStatus
    {
        GPS_FIX_TYPE_NO_GPS = 0,
        GPS_FIX_TYPE_NO_FIX = 1,
        GPS_FIX_TYPE_2D_FIX = 2,
        GPS_FIX_TYPE_3D_FIX = 3,
        GPS_FIX_TYPE_DGPS = 4,
        GPS_FIX_TYPE_RTK_FLOATR = 5,
        GPS_FIX_TYPE_RTK_FIXEDR = 6,
        GPS_FIX_TYPE_STATIC = 7,
        GPS_FIX_TYPE_PPP = 8
    };
    // GPS卫星数量
    uint8_t gps_num;

    //无人机经度、纬度、高度
    float latitude;
    float longitude;
    float altitude;

    // 无人机状态量：位置、速度、姿态
    float position[3];       // [m]
    float velocity[3];       // [m/s]
    float attitude[3];       // [rad]
    Quaternion attitude_q;   // 四元数
    float attitude_rate[3];  // [rad/s]
    float battery_state;     // 电池状态[v]
    float battery_percetage; // [0-1]

    // 话题名
    std::string topic_name;
};

struct Heartbeat
{
    uint32_t count;
    std::string message;
};

struct UGVState
{
    // 时间戳
    uint32_t secs;
    uint32_t nsecs;

    // 无人车编号
    uint8_t ugv_id;

    // 无人车状态量：位置、速度、姿态
    float position[3];
    float velocity[3];
    float attitude[3];

    // 电池电量
    float battery;

    // 四元数
    Quaternion attitude_q;

    // 话题名
    std::string topic_name;
};

struct ModeSelection
{
    uint8_t mode;
    enum Mode
    {
        UAVBASIC = 1,
        UGVBASIC = 2,
        SWARMCONTROL = 3,
        AUTONOMOUSLANDING = 4,
        OBJECTTRACKING = 5,
        EGOPLANNER = 6,
        TRAJECTOYCONTROL = 7,
        CUSTOMMODE = 8,
        REBOOTNX = 9,
        EXITNX = 10
    };
//    bool is_simulation;
    std::vector<int8_t> selectId;
    // uint8_t swarm_num;

    uint8_t use_mode;
    enum UseMode
    {
        // 可能和其他的一些命名存在一些冲突，所以加前缀
        UM_CREATE = 0,
        UM_DELETE = 1
    };

    bool is_simulation;

    int swarm_num;

    std::string cmd;
};

struct Point
{
    double x;
    double y;
    double z;
};

// SwarmCommand.msg
// /prometheus/swarm_command
struct SwarmCommand
{
    // 消息来源
    std::string source;

    //编队套件数量
    uint8_t swarm_num;

    //定位来源
    uint8_t swarm_location_source;
    enum SwarmLocationSource
    {
        mocap = 0,
        gps = 4,
        rtk = 5,
        uwb = 6
    };

    // 命令
    uint8_t Swarm_CMD;
    // enum CMD 控制模式枚举
    // enum CMD 控制模式枚举
    enum SwarmCMD
    {
        Ready = 0,
        Init = 1,
        Start = 2,
        Hold = 3,
        Stop = 4,
        Formation = 5,
        Follow = 11,
        Search = 12,
        Attack = 13
    };

    // 编队控制参考量
    float leader_pos[3];
    float leader_vel[2];
    float swarm_size;
    uint8_t swarm_shape;
    enum SwarmShape
    {
        One_column = 0,
        Triangle = 1,
        Square = 2,
        Circular = 3
    };

    // 搜索控制参考量
    float target_area_x_min; // [m]
    float target_area_y_min; // [m]
    float target_area_x_max; // [m]
    float target_area_y_max; // [m]

    // 攻击控制参考量
    float attack_target_pos[3]; // [m]

    std::vector<struct Point> formation_poses;

    // 话题名
    std::string topic_name;
};

// StationFeedback.msg
// /uav*/prometheus/station_feedback
struct TextInfo
{
    enum MessageTypeGrade
    {
        //INFO:正常运行状况下反馈给地面站的信息,例如程序正常启动,状态切换的提示信息等.
        MTG_INFO = 0,
        //WARN:无人机或软件程序出现意外情况,依然能正常启动或继续执行任务,小概率会出现危险状况,例如无人机RTK无法维持退出到GPS,视觉跟踪目标突然丢失重新搜寻目标等.
        MTG_WARN = 1,
        //ERROR:无人机或软件程序出现重大意外情况,无法正常启动或继续执行任务,极有可能会出现危险状况,需要中断任务以及人为接管控制无人机,例如通信中断,无人机定位发散,ROS节点无法正常启动等.
        MTG_ERROR = 2,
        //FATAL:任务执行过程中,软件崩溃或无人机飞控崩溃导致无人机完全失控,需要迅速人为接管控制无人机降落减少炸机损失.
        MTG_FATAL = 3
    };

    int sec;
    uint8_t MessageType;
    std::string Message;

    // 话题名
    std::string topic_name;
};

//参考文件： GimbalState.msg
//订阅话题： ~/gimbal/state
struct GimbalState
{
    uint8_t Id;
    //# 0: 发一句回一句 `# 1: 发一句，一直回复, 摄像头倍数返回将失效``
    uint8_t feedbackMode;
    //mode 0:手动控制 1:home 2:tracking 3:yaw follow 4:hold吊舱跟随无人机 5:search
    uint8_t mode;
    //是否视频录制
    bool isRecording;
    //# 是否开启自动缩放(VisionDiff需要指定面积才能生效) 0:保持 1:放大 2:缩小 3:自动
    uint8_t zoomState;
    // 当前所处倍数
    float zoomVal;
    //roll,pitch,yaw
    float imuAngle[3];
    //Current gimbal joint angles(roll,pitch,yaw), published at 30 Hz.
    float rotorAngle[3];
    //rpy_vel 角速度
    float imuAngleVel[3];
    //rpy_tgt 目标角度
    float rotorAngleTarget[3];

    // 话题名
    std::string topic_name;
};
//参考文件： VisionDiff.msg
//订阅话题： ~/gimbal/track
struct VisionDiff
{
    uint8_t id;
    uint8_t detect;

    uint16_t objectX;
    uint16_t objectY;
    uint16_t objectWidth;
    uint16_t objectHeight;

    uint16_t frameWidth;
    uint16_t frameHeight;

    // Gimbal 跟踪pid
    float kp;
    float ki;
    float kd;

    int8_t ctlMode; // 0: yaw+pitch, 1: roll+pitch 3:混合(未实现)
    enum CtlMode
    {
        yawPitch = 0,
        rollPitch = 1,
        mix = 3
    };

    float currSize; //框选近大远小
    float maxSize;
    float minSize; //框选大小

    float trackIgnoreError;
    bool autoZoom;
    
    // 话题名
    std::string topic_name;
};

//参考文件： GimbalControl.msg
//订阅话题： ~/gimbal/control
struct GimbalControl
{
    uint8_t Id;
    //control mode 0:nothong 1:angle 2:speed 3:home postion
    uint8_t rpyMode;
    enum RPYMode
    {
        manual = 1,
        home = 2,
        hold = 3,
        fellow = 4
    };

    uint8_t roll;
    uint8_t yaw;
    uint8_t pitch;

    enum ControlMode
    {
        noCtl = 0,
        velocityCtl = 1,
        angleCtl = 2
    };

    float rValue; // deg 单位
    float yValue; // deg
    float pValue; // deg

    // focus
    uint8_t focusMode; // 默认值
    enum FocusMode
    {
        focusStop = 1,
        focusOut = 2,
        focusIn = 3
    };

    // zoom
    uint8_t zoomMode; // 默认值
    enum ZoomMode
    {
        zoomStop = 1,
        zoomOut = 2,
        zoomIn = 3
    };

    // 话题名
    std::string topic_name;
};

struct GimbalService
{
    uint8_t service;
    enum Service
    {
        search = 1,
        record_video = 2,
        track_mode = 3
    };

    bool data;
};

struct GimbalParamSet
{
    std::string param_id;
    double real;
};

struct WindowPosition
{
    //模式：...
    uint8_t mode;
    enum Mode
    {
        IDLE = 0,
        RECTANGLE = 1,
        POINT = 2,
        TRACK_ID = 3,
        FRAME_ID_AND_POINT = 4
    };

    //波门位置X,//波门位置Y(kcf,点击画面的功能的时候使用),左上角为（0,0）
    int16_t origin_x;
    int16_t origin_y;
    int16_t width;
    int16_t height;

    //波门位置X,//波门位置Y
    //int16 window_position_x = origin_x + width/2
    //int16 window_position_y = origin_y + height/2
    int16_t window_position_x;
    int16_t window_position_y;

    //算法检测结果的ID
    int32_t track_id;

    // //被点击或框选帧（画面暂停的ID）
    // int32_t frame_id;

    std::string udp_msg;

    std::string topic_name;
};

//ROS话题: "/deepsort_ros/object_detection_result"
struct DetectionInfo
{
    //目标框的位置（主要斜对角两个点）
    float left;
    float top;
    float bot;
    float right;

    //TRACK TARGET
    int32_t trackIds;
};
struct MultiDetectionInfo
{
    //模式：0：空闲  2.simaRPN  3.deepsort/sort
    uint8_t mode;

    //检测到的目标数量
    int32_t num_objs;

    //每个目标的检测结果
    //DetectionInfo[] detection_infos;
    std::vector<DetectionInfo> detection_infos;
};

struct UGVCommand
{
    // 时间戳
    uint32_t secs;
    uint32_t nsecs;

    // 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
    uint32_t Command_ID;

    // 控制命令的模式
    uint8_t Mode;
    // 控制命令模式的枚举
    enum MODE
    {
        Hold = 0,
        Direct_Control_BODY = 1,
        Direct_Control_ENU = 2,
        Point_Control = 3,
        Path_Control = 4,
        Test = 5
    };

    // 期望位置[m]
    float position_ref[2];
    // 期望偏航[rad]
    float yaw_ref;

    // [m/s]
    float linear_vel[2];
    // [rad/s]
    float angular_vel;

    // 话题名
    std::string topic_name;
};

struct ImageData
{
    std::string name;
    std::string data;
};

struct UAVCommand
{
    //时间戳
    uint32_t secs;
    uint32_t nsecs;

    //控制命令的模式
    uint8_t Agent_CMD;
    //Agent_CMD 枚举
    enum AgentCMD
    {
        Init_Pos_Hover = 1,
        Current_Pos_Hover = 2,
        Land = 3,
        Move = 4,
        User_Mode1 = 5
    };

    // 控制等级 
    uint8_t Control_Level;
    // Control_Level 枚举
    enum ControlLevel
    {
        DEFAULT_CONTROL = 0,        // 默认控制
        ABSOLUTE_CONTROL = 1,       // 绝对控制，在该控制等级下不在响应 默认控制 下任何指令，持续的Move模式应谨慎使用该优先级
        EXIT_ABSOLUTE_CONTROL = 2   // 退出绝对控制控制，在该控制下，会响应上述两种控制的指令
    };

    //移动命令下的子模式
    uint8_t Move_mode;
    // 移动命令下的子模式枚举
    enum MoveMode
    {
        XYZ_POS = 0,           // 惯性系定点控制
        XY_VEL_Z_POS = 1,      // 惯性系定高速度控制
        XYZ_VEL = 2,           // 惯性系速度控制
        XYZ_POS_BODY = 3,      // 机体系位置控制
        XYZ_VEL_BODY = 4,      // 机体系速度控制
        XY_VEL_Z_POS_BODY = 5, // 机体系定高速度控制
        TRAJECTORY = 6,        // 轨迹追踪控制
        XYZ_ATT = 7,           // 姿态控制（来自外部控制器）
        LAT_LON_ALT = 8        // 绝对坐标系下的经纬度
    };
    // 控制参考量
    float position_ref[3];     // [m]
    float velocity_ref[3];     // [m/s]
    float acceleration_ref[3]; // [m/s^2]
    float yaw_ref;             // [rad]
    bool Yaw_Rate_Mode;        // True 代表控制偏航角速率
    float yaw_rate_ref;        // [rad/s]
    float att_ref[4];          // [rad] + [0-1]
    double latitude;           // 无人机经度、纬度、高度
    double longitude;          // 无人机经度、纬度、高度
    double altitude;           // 无人机经度、纬度、高度

    // 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
    uint32_t Command_ID;

    // 话题名
    std::string topic_name;
};

struct UAVSetup
{
    //Setup类型(模拟遥控器)
    uint8_t cmd;
    enum CMD
    {
        ARMING = 0,
        SET_PX4_MODE = 1,
        REBOOT_PX4 = 2,
        SET_CONTROL_MODE = 3,
    };

    bool arming;
    // http://wiki.ros.org/mavros/CustomModes ,可参考该网址设置模式名,常用模式名:OFFBOARD,AUTO.LAND,AUTO.RTL,POSCTL
    std::string px4_mode;
    std::string control_state;

    std::string topic_name;
};

struct UAVControlState
{
    // 无人机编号
    uint8_t uav_id;

    // 无人机控制状态
    uint8_t control_state;
    // 状态枚举
    enum ControlState
    {
        INIT = 0,
        MANUAL_CONTROL = 1,
        HOVER_CONTROL = 2,
        COMMAND_CONTROL = 3,
        LAND_CONTROL = 4
    };

    // 无人机控制器标志量
    uint8_t pos_controller;
    // 状态枚举
    enum ControllerFlag
    {
        PX4_ORIGIN = 0,
        PID = 1,
        UDE = 2,
        NE = 3
    };

    // 无人机安全保护触发标志量
    bool failsafe;

    // 话题名
    std::string topic_name;
};

struct Bspline
{
    int drone_id;
    int order;
    long traj_id;
    double sec;
    std::vector<double> knots;
    std::vector<Point> pos_pts;
    std::vector<double> yaw_pts;
    double yaw_dt;
};

struct MultiBsplines
{
    int drone_id_from;
    std::vector<Bspline> traj;
};

struct Param
{
    uint8_t type;
    enum Type
    {
        INT = 1,
        LONG = 2,
        FLOAT = 3,
        DOUBLE = 4,
        STRING = 5,
        BOOLEAN = 6
    };
    std::string param_name;
    std::string param_value;
};

struct ParamSettings
{
    uint8_t param_module;
    enum ParamModule
    {
        UAVCONTROL = 1,
        UAVCOMMUNICATION = 2,
        SWARMCONTROL = 3,
        UAVCOMMANDPUB = 4
    };
    std::vector<Param> params;
};

struct BasicDataTypeAndValue
{
    std::string name;
    uint8_t type;
    enum Type
    {
        INTEGER = 1,
        BOOLEAN = 2,
        FLOAT = 3,
        DOUBLE = 4,
        STRING = 5
    };
    std::string value;
};
//自定义消息1：地面站->机载端，此处为固定内容，即不能随意更改结构体
struct CustomDataSegment_1
{
    std::vector<BasicDataTypeAndValue> datas;
};

struct ConnectState
{
    uint8_t num;
    bool state;
};

struct Goal
{
    int seq;
    std::string frame_id;
    double position_x;
    double position_y;
    double position_z;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
};

#endif
