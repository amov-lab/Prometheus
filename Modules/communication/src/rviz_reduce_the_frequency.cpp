#include "rviz_reduce_the_frequency.hpp"

ReduceTheFrequency::ReduceTheFrequency(ros::NodeHandle &nh)
{
    int drone_id_;
    nh.param("ROBOT_ID", drone_id_, 1);

    // 订阅需要降低频率的话题 /camera/depth/color/points
    octomap_point_cloud_centers_sub_ = nh.subscribe("/octomap_point_cloud_centers", 100, &ReduceTheFrequency::octomapPointCloudCentersCb, this);
    occupancy_inflate_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/grid_map/occupancy_inflate", 10 , &ReduceTheFrequency::occupancyInflateCb, this);
    scan_sub_ = nh.subscribe("/scan",10 , &ReduceTheFrequency::scanCb, this);
    scan_filtered_sub_ = nh.subscribe("/scan_filtered",10 , &ReduceTheFrequency::scanFilteredCb, this);

    trajectory_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory", 10, &ReduceTheFrequency::trajectoryCb, this);

    odom_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/odom", 10, &ReduceTheFrequency::odomCb, this);

    tf_sub_ = nh.subscribe("/tf",10 , &ReduceTheFrequency::tfCb, this);

    uav_mesh_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh",10 , &ReduceTheFrequency::UAVMeshCb, this);

    optimal_list_sub_ = nh.subscribe("/uav" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list", 10 , &ReduceTheFrequency::optimalListCb, this);

    // 发布降低频率后的话题
    octomap_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers/reduce_the_frequency", 100);
    octomap_compressed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers/reduce_the_frequency/compressed", 100);
    // 点云数据降低频率
    occupancy_inflate_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(drone_id_) + "_ego_planner_node/grid_map/occupancy_inflate/reduce_the_frequency", 100);
    // 点云数据降低频率并过滤
    occupancy_inflate_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(drone_id_) + "_ego_planner_node/grid_map/occupancy_inflate/reduce_the_frequency/filtered", 100);
    // 点云数据降低频率并压缩
    occupancy_inflate_compressed_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(drone_id_) + "_ego_planner_node/grid_map/occupancy_inflate/reduce_the_frequency/compressed", 100);
    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan/reduce_the_frequency", 100);
    scan_filtered_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan_filtered/reduce_the_frequency", 100);
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("/uav" + std::to_string(drone_id_) + "/prometheus/trajectory/reduce_the_frequency", 100);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/uav" + std::to_string(drone_id_) + "/prometheus/odom/reduce_the_frequency", 100);
    tf_pub_ = nh.advertise<tf2_msgs::TFMessage>("/tf/reduce_the_frequency", 100);
    uav_mesh_pub_ = nh.advertise<visualization_msgs::Marker>("/uav" + std::to_string(drone_id_) + "/prometheus/uav_mesh/reduce_the_frequency", 100);
    optimal_list_pub_ = nh.advertise<visualization_msgs::Marker>("/uav" + std::to_string(drone_id_) + "_ego_planner_node/optimal_list/reduce_the_frequency", 100);

    send_timer_1000MS = nh.createTimer(ros::Duration(1.0), &ReduceTheFrequency::send1000MS, this);
    usleep(10000);
    send_timer_500MS = nh.createTimer(ros::Duration(0.5), &ReduceTheFrequency::send500MS, this);
    usleep(10000);
    send_timer_200MS = nh.createTimer(ros::Duration(0.2), &ReduceTheFrequency::send200MS, this);
    usleep(10000);
    send_timer_50MS = nh.createTimer(ros::Duration(0.05), &ReduceTheFrequency::send50MS, this);
    // /uav1_ego_planner_node/optimal_list
}

ReduceTheFrequency::~ReduceTheFrequency()
{

}

void ReduceTheFrequency::send1000MS(const ros::TimerEvent &time_event)
{
    if(octomap_point_cloud_ready)
    {
        octomap_compressed_point_cloud = compressed(*msg);
        octomap_pub_.publish(octomap_point_cloud);
        octomap_compressed_pub_.publish(octomap_compressed_point_cloud);
        octomap_point_cloud_ready = false;
    }
    usleep(100000);
    if(occupancy_inflate_ready)
    {
        occupancy_inflate_filtered_point_cloud = filtered(*msg);
        occupancy_inflate_compressed_point_cloud = compressed(*msg);
        occupancy_inflate_pub_.publish(occupancy_inflate_point_cloud);
        occupancy_inflate_filtered_pub_.publish(occupancy_inflate_filtered_point_cloud);
        occupancy_inflate_compressed_pub_.publish(occupancy_inflate_compressed_point_cloud);
        occupancy_inflate_ready = false;
    }
    usleep(100000);
    if(scan_ready)
    {
        scan_pub_.publish(scan);
        scan_ready = false;
    }
    usleep(100000);
    if(scan_filtered_ready)
    {
        scan_filtered_pub_.publish(scan_filtered);
        scan_filtered_ready = false;
    }
}

void ReduceTheFrequency::send500MS(const ros::TimerEvent &time_event)
{
    if(trajectory_ready)
    {
        trajectory_pub_.publish(trajectory);
        trajectory_ready = false;
    }
    usleep(100000);
    if(uav_mesh_ready)
    {
        uav_mesh_pub_.publish(uav_mesh);
        uav_mesh_ready = false;
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
    compressed_cloud_msg.row_step = compressed_size;  // 压缩数据的大小作为行步长
    compressed_cloud_msg.is_dense = true;
    compressed_cloud_msg.data = std::vector<uint8_t>(compressed_data_ptr, compressed_data_ptr + compressed_size);

    // 释放动态分配的内存
    delete[] compressed_data_ptr;// 获取压缩数据的大小

    return compressed_cloud_msg;
}