#include <occupy_map.h>

namespace Swarm_Planning
{
// 初始化函数
void Occupy_map::init(ros::NodeHandle& nh)
{
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
    // 天花板、地板高度
    nh.param("map/ceil_height_", ceil_height_, 4.9);
    nh.param("map/floor_height_", floor_height_, 0.1);

    // 发布膨胀后的点云
    inflate_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning/global_inflate_cloud", 1);

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
}
// 地图更新函数
void Occupy_map::map_update(const sensor_msgs::PointCloud2ConstPtr & global_point)
{
    global_env_ = global_point;
    has_global_point = true;
}

// 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
void Occupy_map::inflate_point_cloud(void)
{
    if(!has_global_point)
    { 
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Occupy_map [inflate point cloud]: don't have global point, can't inflate!\n");
        return;
    }

    //记录开始时间
    ros::Time time_start = ros::Time::now();

    // 更新的全局点云
    pcl::PointCloud<pcl::PointXYZ> latest_global_cloud_;
    pcl::fromROSMsg(*global_env_, latest_global_cloud_);

    if ((int)latest_global_cloud_.points.size() == 0)  return;

    pcl::PointCloud<pcl::PointXYZ> cloud_inflate_vis_;
    cloud_inflate_vis_.clear();

    //  printf("point size: %d, ifn_num: %d\n", latest_global_cloud_.points.size(), ifn);

    // 创建tf的监听器
    // 这个TF转换是否使用到？
    tf::TransformListener listener;
    try{
        // 等待获取监听信息base_link和base_laser
        listener.waitForTransform("world", "map", ros::Time(0), ros::Duration(3.0));
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }

    // iterate the map
    pcl::PointCloud<pcl::PointXYZ> world_pcl;
    geometry_msgs::PointStamped world_map_point;

    // 膨胀格子数 = 膨胀距离/分辨率
    // ceil返回大于或者等于指定表达式的最小整数
    const int ifn = ceil(inflate_ * inv_resolution_);

    pcl::PointXYZ pt, pt_inf;
    Eigen::Vector3d p3d, p3d_inf;

    // 遍历全局点云中的所有点
    for (size_t i = 0; i < latest_global_cloud_.points.size(); ++i) 
    {
        // 取出第i个点
        pt = latest_global_cloud_.points[i];
        p3d(0) = pt.x;
        p3d(1) = pt.y;
        p3d(2) = pt.z;

        // 如果点云并非world系，则进行左边转换，并存入pt
        // 会有这种情况吗？
        if(global_env_->header.frame_id != "world")
        {
            geometry_msgs::PointStamped map_point;
            map_point.header.frame_id = "map";
            map_point.point.x = pt.x; 
            map_point.point.y = pt.y; 
            map_point.point.z = pt.z; 

            listener.transformPoint("world", map_point, world_map_point);
            printf("** map_point 3d: [%f,  %f,  %f],   world_map_point: [%f, %f, %f]\n", map_point.point.x, map_point.point.y, map_point.point.z, 
            world_map_point.point.x, world_map_point.point.y, world_map_point.point.z);

            pt = pcl::PointXYZ(world_map_point.point.x, world_map_point.point.y, world_map_point.point.z);
        }

        // 根据膨胀距离，膨胀该点
        for (int x = -ifn; x <= ifn; ++x)
            for (int y = -ifn; y <= ifn; ++y)
                for (int z = -ifn; z <= ifn; ++z) 
                {
                    // 为什么Z轴膨胀一半呢？
                    p3d_inf(0) = pt_inf.x = pt.x + x * resolution_;
                    p3d_inf(1) = pt_inf.y = pt.y + y * resolution_;
                    p3d_inf(2) = pt_inf.z = pt.z + 0.5 * z * resolution_;

                    // 若膨胀的点超过天花板或者低于地面，则退出
                    if(p3d_inf(2)>ceil_height_ || p3d_inf(2)<0) break;

                    cloud_inflate_vis_.push_back(pt_inf);

                    // 设置膨胀后的点被占据
                    this->setOccupancy(p3d_inf, 1);
                }
    }

    // cloud_inflate_vis_.width = cloud_inflate_vis_.points.size();
    // cloud_inflate_vis_.height = 1;
    // cloud_inflate_vis_.is_dense = true;
    cloud_inflate_vis_.header.frame_id = "world";

    /* ---------- add ceil  and floor---------- */
    if (ceil_height_ > 0.0) 
    {
        for (double cx = min_range_(0); cx <= max_range_(1); cx += resolution_)
            for (double cy =min_range_(0); cy <= max_range_(1); cy += resolution_) 
            {
                this->setOccupancy(Eigen::Vector3d(cx, cy, ceil_height_), 1);
                cloud_inflate_vis_.push_back(pcl::PointXYZ(cx, cy, ceil_height_));
            }
    }

    if (floor_height_ > 0.0) {
        for (double cx = min_range_(0); cx <= max_range_(1); cx += resolution_)
            for (double cy =min_range_(0); cy <= max_range_(1); cy += resolution_) 
            {
                this->setOccupancy(Eigen::Vector3d(cx, cy, floor_height_), 1);
                cloud_inflate_vis_.push_back(pcl::PointXYZ(cx, cy, floor_height_));
            }
    }

    sensor_msgs::PointCloud2 map_inflate_vis;
    pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);

    inflate_cloud_pub_.publish(map_inflate_vis);

    // 一般需要0.3秒左右（有点久，Astar更新频率是多久呢？ 怎么才能提高膨胀效率呢？）
    printf("inflate global point take %f.\n",   (ros::Time::now()-time_start).toSec());
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

bool Occupy_map::check_safety(Eigen::Vector3d& pos, double check_distance/*, Eigen::Vector3d& map_point*/)
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
