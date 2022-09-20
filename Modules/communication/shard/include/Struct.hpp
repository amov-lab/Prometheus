#ifndef STRUCT_HPP
#define STRUCT_HPP

#include <vector>
#include <cstddef> // NULL
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

//uav control
#define OPENUAVBASIC ""//"gnome-terminal -- roslaunch prometheus_uav_control uav_control_main_outdoor.launch"
// #define CLOSEUAVBASIC "gnome-terminal -- rosnode kill /joy_node | gnome-terminal -- rosnode kill /uav_control_main_1"
//rhea control
#define OPENUGVBASIC ""
#define CLOSEUGVBASIC ""
//集群
#define OPENSWARMCONTROL ""
#define CLOSESWARMCONTROL ""
//自主降落
#define OPENAUTONOMOUSLANDING ""
#define CLOSEAUTONOMOUSLANDING ""
//目标识别与追踪
#define OPENOBJECTTRACKING ""
#define CLOSEOBJECTTRACKING ""
//EGO Planner
#define OPENEGOPLANNER ""
#define CLOSEEGOPLANNER ""

//杀掉除了通信节点和主节点的其他节点
//分为两种情况  
//1:杀掉子模块，这种情况不会杀掉uav control节点和通信节点以及master节点。 
//2:杀掉uav control节点，这种情况下只会保留通信节点以及master节点。
#define CLOSEUAVBASIC ""//"gnome-terminal -- rosnode kill `rosnode list | grep -v /communication_bridge | grep -v /rosout`"
#define CLOSEOTHERMODE ""//"gnome-terminal -- rosnode kill `rosnode list | grep -v /communication_bridge | grep -v /rosout | grep -v /uav_control_main_1 | grep -v /joy_node`"

//重启
#define REBOOTNXCMD "shutdown -r now"
//关机
#define EXITNXCMD "shutdown -h now"

enum MsgId
{
    UAVSTATE = 1,
    TEXTINFO = 3,
    GIMBALSTATE = 4,
    VISIONDIFF = 5,
    HEARTBEAT = 6,
    RHEASTATE = 7,
    MULTIDETECTIONINFO = 8,
    UAVCONTROLSTATE = 9,

    SWARMCOMMAND = 101,
    GIMBALCONTROL = 102,
    GIMBALSERVICE = 103,
    WINDOWPOSITION = 104,
    RHEACONTROL = 105,
    GIMBALPARAMSET = 106,
    IMAGEDATA = 107,
    UAVCOMMAND = 108,
    UAVSETUP = 109,
    PARAMSETTINGS = 110,
    BSPLINE = 111,
    MULTIBSPLINES = 112,

    CONNECTSTATE = 201,
    MODESELECTION = 202,

    //rviz数据
    UGVLASERSCAN = 230,
    UGVPOINTCLOUND2 = 231,

    UGVTFMESSAGE = 232,
    UGVTFSTATIC = 233,

    UGVMARKERARRAY = 234,
    UGVMARKERARRAYLANDMARK = 235,
    UGVMARKERARRAYTRAJECTORY = 236
};

//参考文件： UAVState.msg
//订阅话题： /uav*/prometheus/state
struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & x;
        ar & y;
        ar & z;
        ar & w;
    }
};
//MSG 1
struct UAVState
{
    //无人机编号
    uint8_t uav_id;

    // //无人机状态
    // uint8_t state;
    // //enum agent状态枚举
    // enum State
    // {
    //     unknown = 0,
    //     ready = 1,
    //     dead = 2,
    //     lost = 3
    // };

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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & uav_id;
        // ar & state;
        ar & location_source;
        ar & connected;
        ar & mode;
        ar & armed;
        ar & odom_valid;
        ar & gps_status;
        ar & latitude;
        ar & longitude;
        ar & altitude;
        ar & position;
        ar & velocity;
        ar & attitude;
        ar & attitude_q;
        ar & attitude_rate;
        ar & battery_state;
        ar & battery_percetage;
    }
};

struct Heartbeat
{
    uint32_t count;
    std::string message;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & count;
        ar & message;
    }
};

struct RheaState
{
    uint8_t rhea_id;
    double linear;
    double angular;
    double yaw;
    float latitude;
    float longitude;
    float altitude;
    float position[3];
    float battery_voltage;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & rhea_id;
        ar & linear;
        ar & angular;
        ar & yaw;
        ar & latitude;
        ar & longitude;
        ar & altitude;
        ar & position;
        ar & battery_voltage;
    }
};

struct ModeSelection
{
    uint8_t mode;
    enum Mode
    {
        UAVBASIC = 1,
        UGVBASIC = 2,
        SWARMCONTROL = 3,
        //GIMBAL?
        AUTONOMOUSLANDING = 4,
        OBJECTTRACKING = 5,
        EGOPLANNER = 6,
        CUSTOMMODE = 7,
        REBOOTNX = 8,
        EXITNX = 9
    };
//    bool is_simulation;
    std::vector<uint8_t> selectId;
    // uint8_t swarm_num;

    uint8_t use_mode;
    enum UseMode
    {
        CREATE = 0,
        DELETE = 1
    };

    bool is_simulation;

    int swarm_num;

    std::string cmd;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & selectId;
        ar & mode;
        ar & use_mode;
        ar & is_simulation;
        ar & swarm_num;
        ar & cmd;
    }
};

struct Point
{
    double x;
    double y;
    double z;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & x;
        ar & y;
        ar & z;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & source;
        ar & swarm_num;
        ar & swarm_location_source;
        ar & Swarm_CMD;
        ar & leader_pos;
        ar & leader_vel;
        ar & swarm_size;
        ar & swarm_shape;
        ar & target_area_x_min;
        ar & target_area_y_min;
        ar & target_area_x_max;
        ar & target_area_y_max;
        ar & attack_target_pos;
        ar & formation_poses;
    }
};

// StationFeedback.msg
// /uav*/prometheus/station_feedback
struct TextInfo
{
    enum MessageTypeGrade
    {
        //INFO:正常运行状况下反馈给地面站的信息,例如程序正常启动,状态切换的提示信息等.
        INFO = 0,
        //WARN:无人机或软件程序出现意外情况,依然能正常启动或继续执行任务,小概率会出现危险状况,例如无人机RTK无法维持退出到GPS,视觉跟踪目标突然丢失重新搜寻目标等.
        WARN = 1,
        //ERROR:无人机或软件程序出现重大意外情况,无法正常启动或继续执行任务,极有可能会出现危险状况,需要中断任务以及人为接管控制无人机,例如通信中断,无人机定位发散,ROS节点无法正常启动等.
        ERROR = 2,
        //FATAL:任务执行过程中,软件崩溃或无人机飞控崩溃导致无人机完全失控,需要迅速人为接管控制无人机降落减少炸机损失.
        FATAL = 3
    };

    int sec;
    uint8_t MessageType;
    std::string Message;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & sec;
        ar & MessageType;
        ar & Message;
    }
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
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & Id;
        ar & feedbackMode;
        ar & mode;
        ar & isRecording;
        ar & zoomState;
        ar & zoomVal;
        ar & imuAngle;
        ar & rotorAngle;
        ar & imuAngleVel;
        ar & rotorAngleTarget;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & id;
        ar & objectX;
        ar & objectY;
        ar & objectWidth;
        ar & objectHeight;
        ar & frameWidth;
        ar & frameHeight;
        ar & kp;
        ar & ki;
        ar & kd;
        ar & ctlMode;
        ar & currSize;
        ar & maxSize;
        ar & minSize;
        ar & trackIgnoreError;
        ar & autoZoom;
    }
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
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & Id;
        ar & rpyMode;
        ar & roll;
        ar & yaw;
        ar & pitch;
        ar & rValue;
        ar & yValue;
        ar & pValue;
        ar & focusMode;
        ar & zoomMode;
    }
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
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & service;
        ar & data;
    }
};

struct GimbalParamSet
{
    std::string param_id;
    double real;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & param_id;
        ar & real;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & mode;
        ar & origin_x;
        ar & origin_y;
        ar & width;
        ar & height;
        ar & window_position_x;
        ar & window_position_y;
        ar & track_id;
        // ar & frame_id;
        ar & udp_msg;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & left;
        ar & top;
        ar & bot;
        ar & right;
        ar & trackIds;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & mode;
        ar & num_objs;
        ar & detection_infos;
    }
};

struct RheaGPS
{
    double latitude;
    double longitude;
    double altitude;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & latitude;
        ar & longitude;
        ar & altitude;
    }
};

struct RheaControl
{

    uint8_t Mode;

    //控制模式类型枚举
    enum Mode
    {
        Stop = 0,
        Forward = 1,
        Left = 2,
        Right = 3,
        Back = 4,
        CMD = 5,
        Waypoint = 6
    };

    double linear;
    double angular;

    std::vector<struct RheaGPS> waypoint;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & Mode;
        ar & linear;
        ar & angular;
        ar & waypoint;
    }
};

struct ImageData
{
    std::string name;
    std::string data;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & name;
        ar & data;
    }
};

struct UAVCommand
{
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
    uint Command_ID;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & Agent_CMD;
        ar & Move_mode;
        ar & position_ref;
        ar & velocity_ref;
        ar & acceleration_ref;
        ar & yaw_ref;
        ar & Yaw_Rate_Mode;
        ar & yaw_rate_ref;
        ar & att_ref;
        ar & Command_ID;
        ar & latitude;
        ar & longitude;
        ar & altitude;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & cmd;
        ar & arming;
        ar & px4_mode;
        ar & control_state;
    }
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

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & uav_id;
        ar & control_state;
        ar & pos_controller;
        ar & failsafe;
    }
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
    
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & drone_id;
        ar & order;
        ar & traj_id;
        ar & sec;
        ar & knots;
        ar & pos_pts;
        ar & yaw_pts;
        ar & yaw_dt;
    }
};

struct MultiBsplines
{
    int drone_id_from;
    std::vector<Bspline> traj;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & drone_id_from;
        ar & traj;
    }
};

struct Param
{
    uint8_t type;
    enum Type
    {
        INT = 0,
        LONG = 1,
        FLOAT = 2,
        DOUBLE = 3,
        STRING = 4
    };
    std::string param_name;
    std::string param_value;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & type;
        ar & param_name;
        ar & param_value;
    }
};

struct ParamSettings
{
    int param_nums;
    std::vector<Param> params;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & param_nums;
        ar & params;
    }
};

struct ConnectState
{
    uint8_t num;
    bool state;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* file_version */)
    {
        ar & num;
        ar & state;
    }
};

#endif