#include "rviz_reduce_the_frequency.hpp"

ReduceTheFrequency::ReduceTheFrequency(ros::NodeHandle &nh)
{
    int drone_id_;
    nh.param("ROBOT_ID", drone_id_, 1);

    // 订阅需要降低频率的话题 /camera/depth/color/points
    octomap_point_cloud_centers_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/octomap_point_cloud_centers", 100, &ReduceTheFrequency::octomapPointCloudCentersCb, this);
    occupancy_inflate_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/grid_map/occupancy_inflate", 10 , &ReduceTheFrequency::occupancyInflateCb, this);
    scan_sub_ = nh.subscribe("/scan",10 , &ReduceTheFrequency::scanCb, this);
    scan_filtered_sub_ = nh.subscribe("/scan_filtered",10 , &ReduceTheFrequency::scanFilteredCb, this);

    trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory", 10, &ReduceTheFrequency::trajectoryCb, this);

    odom_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/odom", 10, &ReduceTheFrequency::odomCb, this);

    tf_sub_ = nh.subscribe("/tf",10 , &ReduceTheFrequency::tfCb, this);

    uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh",10 , &ReduceTheFrequency::UAVMeshCb, this);

    optimal_list_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::optimalListCb, this);

    // 无人车相关
    ugv_point_cloud_sub_ = nh.subscribe("/ugv" + std::to_string(drone_id_) + "/cloud_registered",10,&ReduceTheFrequency::ugvPointCloudCb, this);
    ugv_odom_sub_ = nh.subscribe("/ugv" + std::to_string(drone_id_) + "/Odometry",10,&ReduceTheFrequency::ugvOdomCb, this);
    ugv_path_sub_ = nh.subscribe("/ugv" + std::to_string(drone_id_) + "/path",10,&ReduceTheFrequency::ugvPathCb, this);
    ugv_optimal_list_sub_ = nh.subscribe("/ugv" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::ugvOptimalListCb, this);


    // 无人车相关
    ugv_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ugv" + std::to_string(drone_id_) + "/cloud_registered/reduce_the_frequency/compressed", 100);
    ugv_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/ugv" + std::to_string(drone_id_) + "/Odometry/reduce_the_frequency", 100);
    ugv_path_pub_ = nh.advertise<nav_msgs::Path>("/ugv" + std::to_string(drone_id_) + "/path/reduce_the_frequency", 100);
    ugv_optimal_list_pub_ = nh.advertise<visualization_msgs::Marker>("/ugv" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list/reduce_the_frequency", 100);

    
    // /uav1_ego_planner_node/optimal_list
}

ReduceTheFrequency::ReduceTheFrequency(ros::NodeHandle &nh, ReduceTheFrequencyType type, int id)
{
    this->reduce_the_frequency_type = type;
    if(type == ReduceTheFrequencyType_UAV)
    {
        int uav_id = id;

        // 订阅需要降低频率的话题
        // 雷达数据
        //scan_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "/scan",10 , &ReduceTheFrequency::scanCb, this);
        //scan_filtered_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "/scan_filtered",10 , &ReduceTheFrequency::scanFilteredCb, this);
        // octomap数据
        octomap_point_cloud_centers_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "/octomap_point_cloud_centers", 100, &ReduceTheFrequency::octomapPointCloudCentersCb, this);
        // 膨胀层数据
        occupancy_inflate_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "_ego_planner_node/grid_map/occupancy_inflate", 10 , &ReduceTheFrequency::occupancyInflateCb, this);
        // 无人机轨迹数据
        trajectory_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "/prometheus/trajectory", 10, &ReduceTheFrequency::trajectoryCb, this);
        // 无人机里程计数据
        // odom_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "/prometheus/odom", 10, &ReduceTheFrequency::odomCb, this);
        // tf数据
        // tf_sub_ = nh.subscribe("/tf",10 , &ReduceTheFrequency::tfCb, this);
        // 无人机模型数据
        uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "/prometheus/uav_mesh",10 , &ReduceTheFrequency::UAVMeshCb, this);
        // 无人机规划路径数据
        optimal_list_sub_ = nh.subscribe("/uav" + std::to_string(uav_id) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::optimalListCb, this);

        // 发布降低频率后的话题
        // octomap降频后数据
        octomap_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/octomap_point_cloud_centers/reduce_the_frequency", 100);
        // octomap降频压缩后数据
        octomap_compressed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/octomap_point_cloud_centers/reduce_the_frequency/compressed", 100);
        // 膨胀层降低频率
        occupancy_inflate_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "_ego_planner_node/grid_map/occupancy_inflate/reduce_the_frequency", 100);
        // 膨胀层降低频率并过滤
        occupancy_inflate_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "_ego_planner_node/grid_map/occupancy_inflate/reduce_the_frequency/filtered", 100);
        // 膨胀层降低频率并压缩
        occupancy_inflate_compressed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "_ego_planner_node/grid_map/occupancy_inflate/reduce_the_frequency/compressed", 100);
        // scan降频后发布
        // scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/uav" + std::to_string(uav_id) + "/scan/reduce_the_frequency", 100);
        // scan_filtered_pub_ = nh.advertise<sensor_msgs::LaserScan>("/uav" + std::to_string(uav_id) + "/scan_filtered/reduce_the_frequency", 100);
        // 无人机轨迹降频后发布
        trajectory_pub_ = nh.advertise<nav_msgs::Path>("/uav" + std::to_string(uav_id) + "/prometheus/trajectory/reduce_the_frequency", 100);
        // 无人机里程计降频后发布
        // odom_pub_ = nh.advertise<nav_msgs::Odometry>("/uav" + std::to_string(uav_id) + "/prometheus/odom/reduce_the_frequency", 100);
        // 无人机tf数据降频后发布
        // tf_pub_ = nh.advertise<tf2_msgs::TFMessage>("/tf/reduce_the_frequency", 100);
        // 无人机模型降频后发布
        uav_mesh_pub_ = nh.advertise<visualization_msgs::Marker>("/uav" + std::to_string(uav_id) + "/prometheus/uav_mesh/reduce_the_frequency", 100);
        // 无人机规划降频后发布
        optimal_list_pub_ = nh.advertise<visualization_msgs::Marker>("/uav" + std::to_string(uav_id) + "_ego_planner_node/optimal_list/reduce_the_frequency", 100);
    }else if(type == ReduceTheFrequencyType_UGV)
    {
        int ugv_id = id;

        // 无人车相关
        ugv_point_cloud_sub_ = nh.subscribe("/ugv" + std::to_string(ugv_id) + "/cloud_registered",10,&ReduceTheFrequency::ugvPointCloudCb, this);
        ugv_odom_sub_ = nh.subscribe("/ugv" + std::to_string(ugv_id) + "/Odometry",10,&ReduceTheFrequency::ugvOdomCb, this);
        ugv_path_sub_ = nh.subscribe("/ugv" + std::to_string(ugv_id) + "/path",10,&ReduceTheFrequency::ugvPathCb, this);
        ugv_optimal_list_sub_ = nh.subscribe("/ugv" + std::to_string(ugv_id) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::ugvOptimalListCb, this);
    
        ugv_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ugv" + std::to_string(ugv_id) + "/cloud_registered/reduce_the_frequency/compressed", 100);
        ugv_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/ugv" + std::to_string(ugv_id) + "/Odometry/reduce_the_frequency", 100);
        ugv_path_pub_ = nh.advertise<nav_msgs::Path>("/ugv" + std::to_string(ugv_id) + "/path/reduce_the_frequency", 100);
        ugv_optimal_list_pub_ = nh.advertise<visualization_msgs::Marker>("/ugv" + std::to_string(ugv_id) + "_ego_planner_node/optimal_list/reduce_the_frequency", 100);
    }else if(type == ReduceTheFrequencyType_SWARM)
    {
        int is_simulation;
        nh.param<int>("is_simulation", is_simulation, 1);

        // 判断是否仿真
        // 仿真下需要订阅全部无人机数据
        if(is_simulation)
        {
            int swarm_num;
            nh.param<int>("swarm_num", swarm_num, 1);
            for(int i = 0; i < swarm_num; i++)
            {
                // 更换为集群的话题 先暂时不考虑仿真集群情况
                // swarm_optimal_list_sub_.push_back(nh.subscribe("/drone_" + std::to_string(i) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::optimalListCb, this));
                // swarm_occupancy_sub_.push_back(nh.subscribe("/drone_" + std::to_string(i.push_back(nh.subscribe("/drone_" + std::to_string(i) + "_ego_planner_node/grid_map/occupancy", 10 , &ReduceTheFrequency::octomapPointCloudCentersCb, this));) + "_ego_planner_node/grid_map/occupancy", 10 , &ReduceTheFrequency::octomapPointCloudCentersCb, this));
                //swarm_occupancy_inflate_sub_.push_back(nh.subscribe("/drone_" + std::to_string(i.push_back(nh.subscribe("/drone_" + std::to_string(i) + "_ego_planner_node/grid_map/occupancy_inflate", 10 , &ReduceTheFrequency::octomapPointCloudCentersCb, this));) + "_ego_planner_node/grid_map/occupancy", 10 , &ReduceTheFrequency::octomapPointCloudCentersCb, this));
            }
        }
        // 真机下只需要订阅当前无人机数据
        else
        {
            int uav_id = id;

            // 更换为集群的话题
            optimal_list_sub_ = nh.subscribe("/drone_" + std::to_string(uav_id - 1) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::optimalListCb, this);
            octomap_point_cloud_centers_sub_ = nh.subscribe("/drone_" + std::to_string(uav_id - 1) + "_ego_planner_node/grid_map/occupancy", 10 , &ReduceTheFrequency::octomapPointCloudCentersCb, this);
            occupancy_inflate_sub_ = nh.subscribe("/drone_" + std::to_string(uav_id - 1) + "_ego_planner_node/grid_map/occupancy_inflate", 10 , &ReduceTheFrequency::occupancyInflateCb, this);
        
            if(uav_id == 1)
            {
                swarm_graph_visual_sub_ = nh.subscribe("/drone_0_ego_planner_node/swarm_graph_visual", 10 , &ReduceTheFrequency::swarmGraphVisualCb, this);
                swarm_graph_visual_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/drone_0_ego_planner_node/swarm_graph_visual/reduce_the_frequency", 100);
            }
        }
    }

    send_timer_1000MS = nh.createTimer(ros::Duration(1.0), &ReduceTheFrequency::send1000MS, this);
    usleep(10000);
    send_timer_500MS = nh.createTimer(ros::Duration(0.5), &ReduceTheFrequency::send500MS, this);
    usleep(10000);
    send_timer_200MS = nh.createTimer(ros::Duration(0.2), &ReduceTheFrequency::send200MS, this);
    usleep(10000);
    send_timer_50MS = nh.createTimer(ros::Duration(0.05), &ReduceTheFrequency::send50MS, this);
}

ReduceTheFrequency::~ReduceTheFrequency()
{
    // 先停止定时器
    send_timer_1000MS.stop();
    send_timer_500MS.stop();
    send_timer_200MS.stop();
    send_timer_50MS.stop();
    sleep(1); // 等待1s
    // 关闭其他ROS订阅者发布者
    if(this->reduce_the_frequency_type == ReduceTheFrequencyType_UAV)
    {
        octomap_point_cloud_centers_sub_.shutdown();
        occupancy_inflate_sub_.shutdown();
        trajectory_sub_.shutdown();
        uav_mesh_sub_.shutdown();
        optimal_list_sub_.shutdown();
    }else if(this->reduce_the_frequency_type == ReduceTheFrequencyType_UGV)
    {
        ugv_point_cloud_sub_.shutdown();
        ugv_odom_sub_.shutdown();
        ugv_path_sub_.shutdown();
        ugv_optimal_list_sub_.shutdown();
    }else if(this->reduce_the_frequency_type == ReduceTheFrequencyType_SWARM)
    {
        optimal_list_sub_.shutdown();
        octomap_point_cloud_centers_sub_.shutdown();
        occupancy_inflate_sub_.shutdown();
        swarm_graph_visual_sub_.shutdown();
    }
}

void ReduceTheFrequency::send1000MS(const ros::TimerEvent &time_event)
{
    if(octomap_point_cloud_ready)
    {
        octomap_compressed_point_cloud = compressed(octomap_point_cloud);
        octomap_pub_.publish(octomap_point_cloud);
        octomap_compressed_pub_.publish(octomap_compressed_point_cloud);
        octomap_point_cloud_ready = false;
    }
    if(occupancy_inflate_ready)
    {
        occupancy_inflate_filtered_point_cloud = filtered(occupancy_inflate_point_cloud);
        occupancy_inflate_compressed_point_cloud = compressed(occupancy_inflate_point_cloud);
        occupancy_inflate_pub_.publish(occupancy_inflate_point_cloud);
        occupancy_inflate_filtered_pub_.publish(occupancy_inflate_filtered_point_cloud);
        occupancy_inflate_compressed_pub_.publish(occupancy_inflate_compressed_point_cloud);
        occupancy_inflate_ready = false;
    }
    if(scan_ready)
    {
        scan_pub_.publish(scan);
        scan_ready = false;
    }
    if(scan_filtered_ready)
    {
        scan_filtered_pub_.publish(scan_filtered);
        scan_filtered_ready = false;
    }
    if(ugv_point_cloud_ready)
    {
        ugv_compressed_point_cloud = compressed(ugv_point_cloud);
        ugv_point_cloud_pub_.publish(ugv_compressed_point_cloud);
        ugv_point_cloud_ready = false;
    }
}

void ReduceTheFrequency::send500MS(const ros::TimerEvent &time_event)
{
    if(trajectory_ready)
    {
        trajectory_pub_.publish(trajectory);
        trajectory_ready = false;
    }
    if(uav_mesh_ready)
    {
        uav_mesh_pub_.publish(uav_mesh);
        uav_mesh_ready = false;
    }
    if(ugv_odom_ready)
    {
        ugv_odom_pub_.publish(ugv_odom);
        ugv_odom_ready = false;
    }
    if(ugv_path_ready)
    {
        ugv_path_pub_.publish(ugv_path);
        ugv_path_ready = false;
    }
    if(swarm_graph_visual_ready)
    {
        swarm_graph_visual_pub_.publish(swarm_graph_visual);
        swarm_graph_visual_ready = false;
    }
}

void ReduceTheFrequency::send200MS(const ros::TimerEvent &time_event)
{
    if(odom_ready)
    {
        odom_pub_.publish(odom);
        odom_ready = false;
    }
}

void ReduceTheFrequency::send50MS(const ros::TimerEvent &time_event)
{
    if(optimal_list_ready)
    {
        optimal_list_pub_.publish(optimal_list);
        optimal_list_ready = false;
    }
    if(ugv_optimal_list_ready)
    {
        ugv_optimal_list_pub_.publish(ugv_optimal_list);
        ugv_optimal_list_ready = false;
    }
}

void ReduceTheFrequency::octomapPointCloudCentersCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    octomap_point_cloud = *msg;
    octomap_point_cloud_ready = true;
}

void ReduceTheFrequency::trajectoryCb(const nav_msgs::Path::ConstPtr &msg)
{
    trajectory = *msg;
    trajectory_ready = true;
}

void ReduceTheFrequency::scanCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = *msg;
    scan_ready = true;
}

void ReduceTheFrequency::scanFilteredCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan_filtered  = *msg;
    scan_filtered_ready = true;
}

void ReduceTheFrequency::occupancyInflateCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    occupancy_inflate_point_cloud = *msg;
    occupancy_inflate_ready = true;
}

void ReduceTheFrequency::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
    odom_ready = true;
}

void ReduceTheFrequency::tfCb(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    if(msg->transforms[0].header.frame_id == "world" || msg->transforms[0].header.frame_id == "base_link")
    {
        if(tf.transforms.size() == 0) 
        {
            tf = *msg;
            tf_pub_.publish(tf);
        }
        else if(tf.transforms[0].header.frame_id != msg->transforms[0].header.frame_id || tf.transforms[0].child_frame_id != msg->transforms[0].child_frame_id)
        {
            tf = *msg;
            tf_pub_.publish(tf);
        }
    }
}

void ReduceTheFrequency::UAVMeshCb(const visualization_msgs::Marker::ConstPtr &msg)
{   
    uav_mesh = *msg;
    uav_position_x = uav_mesh.pose.position.x;
    uav_position_y = uav_mesh.pose.position.y;
    uav_mesh_ready = true;
}

void ReduceTheFrequency::optimalListCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    optimal_list = *msg;
    optimal_list_ready = true;
}

void ReduceTheFrequency::ugvPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ugv_point_cloud = *msg;
    ugv_point_cloud_ready = true;
}

void ReduceTheFrequency::ugvOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    ugv_odom = *msg;
    ugv_odom_ready = true;
}

void ReduceTheFrequency::ugvPathCb(const nav_msgs::Path::ConstPtr &msg)
{
    ugv_path = *msg;
    ugv_path_ready = true;
}

void ReduceTheFrequency::ugvOptimalListCb(const visualization_msgs::Marker::ConstPtr &msg)
{
    ugv_optimal_list = *msg;
    ugv_optimal_list_ready = true;
}

void ReduceTheFrequency::swarmGraphVisualCb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    swarm_graph_visual = *msg;
    swarm_graph_visual_ready = true;
}

sensor_msgs::PointCloud2 ReduceTheFrequency::filtered(const sensor_msgs::PointCloud2 msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud);

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");  // 根据需要选择过滤的字段
    pass.setFilterLimits(-3 + uav_position_x , 3 + uav_position_x);  // 设置范围
    pass.filter(*cloud);  // 执行滤波操作
    pass.setFilterFieldName("y");  // 根据需要选择过滤的字段
    pass.setFilterLimits(-3 + uav_position_y , 3 + uav_position_y);  // 设置范围
    pass.filter(*cloud);  // 执行滤波操作

    // 转换回 ROS 点云消息
    sensor_msgs::PointCloud2 decompressed_msg;
    pcl::toROSMsg(*cloud, decompressed_msg);
    decompressed_msg.header = msg.header;

    return decompressed_msg;
}

sensor_msgs::PointCloud2 ReduceTheFrequency::compressed(const sensor_msgs::PointCloud2 msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud);

    // 创建Octree压缩对象
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octree_compression;
    octree_compression.setInputCloud(cloud);

    // 压缩点云数据
    std::stringstream compressed_data;
    octree_compression.encodePointCloud(cloud,compressed_data);

    // 获取压缩数据的大小
    std::string compressed_str = compressed_data.str();
    size_t compressed_size = compressed_str.size();

    // 创建动态分配的内存并复制压缩数据
    uint8_t* compressed_data_ptr = new uint8_t[compressed_size];
    std::memcpy(compressed_data_ptr, compressed_str.data(), compressed_size);

    // 创建压缩后的PointCloud2消息
    sensor_msgs::PointCloud2 compressed_cloud_msg;
    compressed_cloud_msg.header = msg.header;
    compressed_cloud_msg.header.stamp = ros::Time::now();
    compressed_cloud_msg.row_step = compressed_size;  // 压缩数据的大小作为行步长
    compressed_cloud_msg.is_dense = true;
    compressed_cloud_msg.data = std::vector<uint8_t>(compressed_data_ptr, compressed_data_ptr + compressed_size);

    // 释放动态分配的内存
    delete[] compressed_data_ptr;// 获取压缩数据的大小

    return compressed_cloud_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reduce_the_frequency");
    ros::NodeHandle nh("~");

    printf("\033[1;32m---->[reduce_the_frequency] start running\n\033[0m");

    int robot_type;// 机器类型
    int id;// 机器ID
    // 获取参数值
    nh.param<int>("robot_type", robot_type, 1);
    nh.param<int>("id", id, 1);

    ReduceTheFrequency reduce_the_frequency_(nh, (ReduceTheFrequencyType)robot_type, id);

    ros::spin();

    return 0;
}