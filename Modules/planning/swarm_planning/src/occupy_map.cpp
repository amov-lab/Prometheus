#include <occupy_map.h>

namespace Swarm_Planning
{
// 初始化函数
void Occupy_map::init(ros::NodeHandle& nh)
{
    nh.param<string>("swarm_planner/uav_name", uav_name, "/uav0");
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("swarm_planner/is_2D", is_2D, true); 
    // 2D规划时,定高高度
    nh.param("swarm_planner/fly_height_2D", fly_height_2D, 1.0);
    // 地图原点
    nh.param("map/origin_x", origin_(0), -5.0);
    nh.param("map/origin_y", origin_(1), -5.0);
    nh.param("map/origin_z", origin_(2), 0.0);
    // 地图实际尺寸，单位：米
    nh.param("map/map_size_x", map_size_3d_(0), 10.0);
    nh.param("map/map_size_y", map_size_3d_(1), 10.0);
    nh.param("map/map_size_z", map_size_3d_(2), 5.0);
    // 地图分辨率，单位：米
    nh.param("map/resolution", resolution_,  0.2);
    // 地图膨胀距离，单位：米
    nh.param("map/inflate", inflate_,  0.3);

    // 发布 地图rviz显示
    global_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(uav_name + "/prometheus/swarm_planning/map/global_pcl",  10); 
    // 发布膨胀后的点云
    inflate_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(uav_name + "/prometheus/swarm_planning/map/global_inflate_pcl", 1);
 
    // 发布二维占据图？
    // 发布膨胀后的二维占据图？

    this->inv_resolution_ = 1.0 / resolution_;
    for (int i = 0; i < 3; ++i)
    {
        // 占据图尺寸 = 地图尺寸 / 分辨率
        grid_size_(i) = ceil(map_size_3d_(i) / resolution_);
    }
        
    // 占据容器的大小 = 占据图尺寸 x*y*z
    occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);

    min_range_ = origin_;
    max_range_ = origin_ + map_size_3d_;   

    // 对于二维情况，重新限制点云高度
    if(is_2D == true)
    {
        min_range_(2) = fly_height_2D - resolution_;
        max_range_(2) = fly_height_2D + resolution_;
    }

    nei_pos1.x = 0.0;
    nei_pos1.y = 0.0;
    nei_pos1.z = -1.0;
    nei_pos2.x = 0.0;
    nei_pos2.y = 0.0;
    nei_pos2.z = -1.0;
}

// 地图更新函数 - 输入：全局点云
void Occupy_map::map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr & global_point)
{
    has_global_point = true;
    global_env_ = global_point;
}

// 地图更新函数 - 输入：局部点云
void Occupy_map::map_update_lpcl(const sensor_msgs::PointCloud2ConstPtr & local_point, const nav_msgs::Odometry & odom)
{
    has_global_point = true;
// 待江涛更新
// 将传递过来的局部点云转为全局点云
}

// 地图更新函数 - 输入：laser
void Occupy_map::map_update_laser(const sensor_msgs::LaserScanConstPtr & local_point, const nav_msgs::Odometry & odom)
{
    has_global_point = true;
// 待更新
// 将传递过来的数据转为全局点云
}

void Occupy_map::setNeiPos(Eigen::Vector3d pos,  int uav_id)
{
    if(uav_id == 0)
    {
        nei_pos1.x = pos[0];
        nei_pos1.y = pos[1];
        nei_pos1.z = pos[2];
        
    }else if(uav_id == 1)
    {
        nei_pos2.x = pos[0];
        nei_pos2.y = pos[1];
        nei_pos2.z = pos[2];
    }
}

// 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
// Astar规划路径时，采用的是此处膨胀后的点云（setOccupancy只在本函数中使用）
void Occupy_map::inflate_point_cloud(void)
{
    if(!has_global_point)
    { 
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Occupy_map [inflate point cloud]: don't have global point, can't inflate!\n");
        return;
    }

    // 发布未膨胀点云
    global_pcl_pub.publish(*global_env_);

    //记录开始时间
    ros::Time time_start = ros::Time::now();

    // 转化为PCL的格式进行处理
    pcl::PointCloud<pcl::PointXYZ> latest_global_cloud_;
    pcl::fromROSMsg(*global_env_, latest_global_cloud_);


    // 增加邻居位置至全局点云
    // if(nei_pos1.z != -1.0)
    // {
    //     latest_global_cloud_.push_back(nei_pos1);
    // }
    // if(nei_pos2.z != -1.0)
    // {
    //     latest_global_cloud_.push_back(nei_pos2);
    // }


    if ((int)latest_global_cloud_.points.size() == 0)  
    {return;}

    pcl::PointCloud<pcl::PointXYZ> cloud_inflate_vis_;
    cloud_inflate_vis_.clear();

    // 膨胀格子数 = 膨胀距离/分辨率
    // ceil返回大于或者等于指定表达式的最小整数
    const int ifn = ceil(inflate_ * inv_resolution_);

    pcl::PointXYZ pt_inf;
    Eigen::Vector3d p3d, p3d_inf;

    // 遍历全局点云中的所有点
    for (size_t i = 0; i < latest_global_cloud_.points.size(); ++i) 
    {
        // 取出第i个点
        p3d(0) = latest_global_cloud_.points[i].x;
        p3d(1) = latest_global_cloud_.points[i].y;
        p3d(2) = latest_global_cloud_.points[i].z;

        // 若取出的点不在地图内（膨胀时只考虑地图范围内的点）
        if(!isInMap(p3d))
        {
            continue;
        }

        // 根据膨胀距离，膨胀该点
        for (int x = -ifn; x <= ifn; ++x)
            for (int y = -ifn; y <= ifn; ++y)
                for (int z = -ifn; z <= ifn; ++z) 
                {
                    // 为什么Z轴膨胀一半呢？ z 轴其实可以不膨胀
                    p3d_inf(0) = pt_inf.x = p3d(0) + x * resolution_;
                    p3d_inf(1) = pt_inf.y = p3d(1) + y * resolution_;
                    p3d_inf(2) = pt_inf.z = p3d(2) + 0.5 * z * resolution_;

                    // 若膨胀的点不在地图内（膨胀时只考虑地图范围内的点）
                    if(!isInMap(p3d_inf))
                    {
                        continue;
                    }

                    cloud_inflate_vis_.push_back(pt_inf);

                    // 设置膨胀后的点被占据
                    this->setOccupancy(p3d_inf, 1);
                }
    }

    const int ifn_nei = 3*ifn;

    // 膨胀邻居
    // 不能这么膨胀，这样被膨胀后的地方 不会消失 会一直被占据！！！
    if(nei_pos1.z != -1.0)
    {
        // 根据膨胀距离，膨胀该点
        for (int x = -ifn_nei; x <= ifn_nei; ++x)
            for (int y = -ifn_nei; y <= ifn_nei; ++y)
                for (int z = -ifn_nei; z <= ifn_nei; ++z) 
                {
                    // 为什么Z轴膨胀一半呢？ z 轴其实可以不膨胀
                    p3d_inf(0) = pt_inf.x = nei_pos1.x + x * resolution_;
                    p3d_inf(1) = pt_inf.y = nei_pos1.y + y * resolution_;
                    p3d_inf(2) = pt_inf.z = nei_pos1.z + 0.5 * z * resolution_;

                    cloud_inflate_vis_.push_back(pt_inf);

                    // 设置膨胀后的点被占据
                    this->setOccupancy(p3d_inf, 1);
                }
    }
    if(nei_pos2.z != -1.0)
    {
        // 根据膨胀距离，膨胀该点
        for (int x = -ifn_nei; x <= ifn_nei; ++x)
            for (int y = -ifn_nei; y <= ifn_nei; ++y)
                for (int z = -ifn_nei; z <= ifn_nei; ++z) 
                {
                    // 为什么Z轴膨胀一半呢？ z 轴其实可以不膨胀
                    p3d_inf(0) = pt_inf.x = nei_pos2.x + x * resolution_;
                    p3d_inf(1) = pt_inf.y = nei_pos2.y + y * resolution_;
                    p3d_inf(2) = pt_inf.z = nei_pos2.z + 0.5 * z * resolution_;

                    cloud_inflate_vis_.push_back(pt_inf);

                    // 设置膨胀后的点被占据
                    this->setOccupancy(p3d_inf, 1);
                }
    }


    cloud_inflate_vis_.header.frame_id = "world";

    // 转化为ros msg发布
    sensor_msgs::PointCloud2 map_inflate_vis;
    pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);

    inflate_pcl_pub.publish(map_inflate_vis);

    static int exec_num=0;
    exec_num++;

    // 此处改为根据循环时间计算的数值
    if(exec_num == 20)
    {
        // 膨胀地图效率与地图大小有关（有点久，Astar更新频率是多久呢？ 怎么才能提高膨胀效率呢？）
        printf("inflate global point take %f [s].\n",   (ros::Time::now()-time_start).toSec());
        exec_num=0;
    }  

}

void Occupy_map::setOccupancy(Eigen::Vector3d pos, int occ) 
{
    if (occ != 1 && occ != 0) 
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "occ value error!\n");
        return;
    }

    if (!isInMap(pos))
    {
        return;
    }

    Eigen::Vector3i id;
    posToIndex(pos, id);

    // 设置为占据/不占据 索引是如何索引的？ [三维地图 变 二维数组]
    // 假设10*10*10米，分辨率1米，buffer大小为 1000 （即每一个占格都对应一个buffer索引）
    // [0.1,0.1,0.1] 对应索引为[0,0,0]， buffer索引为 0  
    // [9.9,9.9,9.9] 对应索引为[9,9,9]， buffer索引为 900+90+9 = 999
    occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] = occ;
}

bool Occupy_map::isInMap(Eigen::Vector3d pos) 
{
    // min_range就是原点，max_range就是原点+地图尺寸
    // 比如设置0,0,0为原点，[0,0,0]点会被判断为不在地图里
    //　同时　对于２Ｄ情况，超出飞行高度的数据也会认为不在地图内部
    if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4) 
    {
        return false;
    }

    if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4) 
    {
        return false;
    }

    return true;
}

bool Occupy_map::check_safety(Eigen::Vector3d& pos, double check_distance)
{
    if(!isInMap(pos))
    {
        // 当前位置点不在地图内
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "[check_safety]: the odom point is not in map\n");
        return 0;
    }
    Eigen::Vector3i id;
    posToIndex(pos, id);
    Eigen::Vector3i id_occ;
    Eigen::Vector3d pos_occ;

    int check_dist_xy = int(check_distance/resolution_);
    int check_dist_z=0;
    int cnt=0;
    for(int ix=-check_dist_xy; ix<=check_dist_xy; ix++){
        for(int iy=-check_dist_xy; iy<=check_dist_xy; iy++){
            for(int iz=-check_dist_z; iz<=check_dist_z; iz++){
                id_occ(0) = id(0)+ix;
                id_occ(1) = id(1)+iy;
                id_occ(2) = id(2)+iz;
                indexToPos(id_occ, pos_occ);
                if(!isInMap(pos_occ)){
                    // printf("[check_safety]: current odom is near the boundary of the map\n");
                    // pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "[check_safety]: current odom is near the boundary of the map\n");
                    return 0;
                }
                if(getOccupancy(id_occ)){
                    // printf("[check_safety]: current state is dagerous, the pos [%d, %d, %d], is occupied\n", ix, iy, iz);
                    cnt++;             
                }
            }
        }
    }
    if(cnt>5){
        return 0;
    }
    return 1;

}

void Occupy_map::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i &id) 
{
    for (int i = 0; i < 3; ++i)
    {
        id(i) = floor((pos(i) - origin_(i)) * inv_resolution_);
    }
       
}

void Occupy_map::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos) 
{
    for (int i = 0; i < 3; ++i)
    {
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
    }
}

int Occupy_map::getOccupancy(Eigen::Vector3d pos) 
{
    if (!isInMap(pos))
    {
        return -1;
    }
        
    Eigen::Vector3i id;
    posToIndex(pos, id);

    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

int Occupy_map::getOccupancy(Eigen::Vector3i id) 
{
    if (id(0) < 0 || id(0) >= grid_size_(0) || id(1) < 0 || id(1) >= grid_size_(1) || id(2) < 0 ||
        id(2) >= grid_size_(2))
    {
        return -1;
    }
        
    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}
}
