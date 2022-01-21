//
// Created by taojiang on 2020/1/8.
//

#include "plan_env/global_point_sdf.h"


namespace dyn_planner {

    void constrains(double &n, double min, double max)
    {
        if(n > max)
            n = max;
        else if(n < min)
            n = min;
    }

    SDFMap_Global::SDFMap_Global(Eigen::Vector3d ori, double resolution, Eigen::Vector3d size) {
        this->origin_ = ori;
        this->resolution_sdf_ = resolution;
        this->resolution_inv_ = 1 / resolution_sdf_;
        this->map_size_ = size;
        for (int i = 0; i < 3; ++i)
            grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
        // cout << "grid num:" << grid_size_.transpose() << endl;
        min_range_ = origin_;
        max_range_ = origin_ + map_size_;
    }

    //判断是否越界
    bool SDFMap_Global::isInMap(Eigen::Vector3d pos) {
        if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4) {
            // ROS_INFO("excess min range: pos: [%f, %f, %f], min range: [%f, %f, %f]", pos(0), pos(1), pos(2), min_range_(0), min_range_(1), min_range_(2));
            return false;
        }
        if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4) {
            // ROS_INFO("excess max range: pos: [%f, %f, %f], max range: [%f, %f, %f]", pos(0), pos(1), pos(2), max_range_(0), max_range_(1), max_range_(2));
            return false;
        }
        return true;
    }

    void SDFMap_Global::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i &id) {
        for (int i = 0; i < 3; ++i)
            id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
    }

    void SDFMap_Global::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos) {
        for (int i = 0; i < 3; ++i)
            pos(i) = (id(i) + 0.5) * resolution_sdf_ + origin_(i);
    }


    // 从sdf图得到最近距离和相应梯度
    double SDFMap_Global::evaluateEDTWithGrad(const Eigen::Vector3d& pos, double& dist,
                               Eigen::Vector3d& grad){
        auto f_sat = [](double a, double sat_l, double sat_h)->double{
            return a < sat_l? a-sat_l: (a>sat_h ? a-sat_h : 0);
        };
        if (!isInMap(pos))
        {
            // printf("[evaluateEDTWithGrad]: pos is not in map!\n");
            dist = 0.0;
            grad(0) = -f_sat(pos(0), min_range_(0), max_range_(0));
            grad(1) = -f_sat(pos(1), min_range_(1), max_range_(1));
            grad(2) = -f_sat(pos(2), min_range_(2), max_range_(2));;
            return -1;
        }

        std::vector<double> location_gradient_query = sdf_map->GetGradient(pos(0), pos(1), pos(2), true);
        grad(0) = location_gradient_query[0];
        grad(1) = location_gradient_query[1];
        grad(2) = location_gradient_query[2];

        std::pair<float, bool> location_sdf_query = sdf_map->GetSafe(pos(0), pos(1), pos(2));
        dist = location_sdf_query.first;
        // printf("esdf gradient: %f, %f, %f\n", grad(0), grad(1), grad(2));
    }

    // 从sdf地图，感知最近距离
    double SDFMap_Global::getDistance(Eigen::Vector3d pos) {

        if (!isInMap(pos)){
            return 0.0;
        }


        // get sdf directly from sdf_tools
//        Eigen::Vector3d ori_pos = pos;
//        constrains(pos(0), -9.8, 9.8);
//        constrains(pos(1), -9.8, 9.8);
//        constrains(pos(2), 0.2, 4.8);
//        std::vector<double> location_gradient_query =
//                this->sdf->GetGradient(pos(0), pos(1), pos(2), true);
//        grad(0) = location_gradient_query[0];
//        grad(1) = location_gradient_query[1];
//        grad(2) = location_gradient_query[2];
        std::pair<float, bool> location_sdf_query = sdf_map->GetSafe(pos(0), pos(1), pos(2));
        double dist = location_sdf_query.first;

        if(dist < 1e-5){
#ifdef DEBUG
              cout << "sdf  get dist   for  pos:" << pos << "with its dist:" << dist  << endl;
#endif    
        }       
        return dist;
    }

    void SDFMap_Global::globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
        /* need odom_ for center radius sensing */
        if (!have_odom_) {

#ifdef DEBUG
            ROS_INFO("global_point_sdf: --- no odom!---");
#endif  
            return;
        }
        
        ros::Time begin_load_pcl = ros::Time::now();
        // ROS_INFO("--- SDFMAP_GLOBAL: have pcl, begin ---");
        pcl::fromROSMsg(*msg, latest_global_cloud_);

         if ((int)latest_global_cloud_.points.size() == 0) return;

        Eigen::Vector3d center(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
        if (isnan(center(0)) || isnan(center(1)) || isnan(center(2)))
            return;

        new_map_ = false;
        has_global_point = true;
        
        pcl::PointXYZ pt, pt_inf;
        Eigen::Vector3d p3d, p3d_inf;
        const int ifn = ceil(inflate_ * resolution_inv_);

        cloud_inflate_vis_.clear();

        // sdf
        sdf_tools::COLLISION_CELL obstacle_cell(1.0);
        vector<Eigen::Vector3d> obstacles;
        ros::Time begin_collision = ros::Time::now();

#ifdef DEBUG
        ROS_INFO("--- SDFMAP_GLABAL: begin collision_map, time: %f", begin_collision.toSec()-begin_load_pcl.toSec());
        ROS_INFO("point size: %d, resolution_sdf_: %f   ifn_num: %d", latest_global_cloud_.points.size(), resolution_sdf_, ifn);
#endif  
        int map_num{0};
        // 创建tf的监听器
        tf::TransformListener listener;
        geometry_msgs::PointStamped world_map_point;
        try{
            // 等待获取监听信息base_link和base_laser
            listener.waitForTransform("world", "map", ros::Time(0), ros::Duration(3.0));
            // ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
            //     laser_point.point.x, laser_point.point.y, laser_point.point.z,
            //     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }

        for (size_t i = 0; i < latest_global_cloud_.points.size(); ++i) {
            pt = latest_global_cloud_.points[i];
            p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
                   
            geometry_msgs::PointStamped map_point;
            map_point.header.frame_id = "map";
            map_point.point.x = pt.x; 
            map_point.point.y = pt.y; 
            map_point.point.z = pt.z; 

            listener.transformPoint("world", map_point, world_map_point);
            // printf("** map_point 3d: [%f,  %f,  %f],   world_map_point: [%f, %f, %f]\n", map_point.point.x, map_point.point.y, map_point.point.z, 
            //  world_map_point.point.x, world_map_point.point.y, world_map_point.point.z);

            pt.x = p3d(0) = world_map_point.point.x;
            pt.y = p3d(1) = world_map_point.point.y;
            pt.z = p3d(2) = world_map_point.point.z;
            /* point inside update range */
            if ((center - p3d).norm() < update_range_ && pt.z < ceil_height_ && pt.z > 0) 
            {
                /* inflate the point */
                // printf("** point 3d: [%f,  %f,  %f] \n", p3d(0), p3d(1), p3d(2));
                // printf("in update range\n");

                for (int x = -ifn; x <= ifn; ++x)
                    for (int y = -ifn; y <= ifn; ++y)
                        for (int z = -ifn; z <= ifn; ++z) {
                            p3d_inf(0) = pt_inf.x = pt.x + x * resolution_sdf_;
                            p3d_inf(1) = pt_inf.y = pt.y + y * resolution_sdf_;
                            p3d_inf(2) = pt_inf.z = pt.z + 0.5 * z * resolution_sdf_;

                            if(p3d_inf(2)>ceil_height_ || p3d_inf(2)<0) 
                            {
                                // printf("** sdf map: height is out of map \n");
                                break;
                            }

                            obstacles.push_back(p3d_inf);
                            collision_map->Set(p3d_inf(0), p3d_inf(1), p3d_inf(2), obstacle_cell);

                            cloud_inflate_vis_.push_back(pt_inf);
                        }
            }
        }
        int num_obstacle_1 = obstacles.size();


        /* ---------- add ceil and floor---------- */
        if (ceil_height_ > 0.0) {
            for (double cx = center(0) - update_range_; cx <= center(0) + update_range_; cx += resolution_sdf_)
                for (double cy = center(1) - update_range_; cy <= center(1) + update_range_; cy += resolution_sdf_) {
                    obstacles.push_back(Eigen::Vector3d(cx, cy, ceil_height_));
                    obstacles.push_back(Eigen::Vector3d(cx, cy, 0.0));
                    collision_map->Set(cx, cy, ceil_height_, obstacle_cell);
                    collision_map->Set(cx, cy, 0.0, obstacle_cell);
                    cloud_inflate_vis_.push_back(pcl::PointXYZ(cx, cy, ceil_height_));
                    cloud_inflate_vis_.push_back(pcl::PointXYZ(cx, cy, 0.0));
                }
        }

        /* ---------- pub inflate cloud point---------- */

        sensor_msgs::PointCloud2 map_inflate_vis;
        pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);
        // map_inflate_vis.header.frame_id = msg->header.frame_id;
        map_inflate_vis.header.frame_id = "world";
        // map_inflate_vis.width = cloud_inflate_vis_.points.size();
        // map_inflate_vis.height = 1;
        // map_inflate_vis.is_dense = true;
        
        map_inflate_vis.header.seq = msg->header.seq;
        map_inflate_vis.header.stamp = msg->header.stamp;

        inflate_cloud_pub_.publish(map_inflate_vis);
#ifdef DEBUG
        int num_obstacle_2 = obstacles.size();
        ROS_INFO("obstacle point number 1:  %d,  2: %d", num_obstacle_1, num_obstacle_2);

        ros::Time endcollisiion_map = ros::Time::now();
        ROS_INFO("--- SDFMAP_GLABAL: begin sdf, time: %f", endcollisiion_map.toSec()-begin_collision.toSec());
#endif  
        // Build the signed distance field
        float oob_value = INFINITY;
        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf_with_extrema =
                collision_map->ExtractSignedDistanceField(oob_value);

        ros::Time end_sdf_time = ros::Time::now();
#ifdef DEBUG
        ROS_INFO("--- SDFMAP_GLABAL: end sdf, time: %f", end_sdf_time.toSec()-endcollisiion_map.toSec());
#endif  

        sdf_map.reset(new sdf_tools::SignedDistanceField(sdf_with_extrema.first));

#ifdef DEBUG
        ROS_INFO("successfully build esdf");
#endif  

        map_valid_ = true;
    }


    void SDFMap_Global::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
        odom_ = *msg;
        odom_.header.frame_id = msg->header.frame_id;  // "world"
        have_odom_ = true;
    }


    void SDFMap_Global::init(ros::NodeHandle &nh) {
        node_ = nh;

        /* ---------- param ---------- */
        node_.param("sdf_map/origin_x", origin_(0), -20.0);
        node_.param("sdf_map/origin_y", origin_(1), -20.0);
        node_.param("sdf_map/origin_z", origin_(2), 0.0);

        node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
        node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
        node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

        node_.param("sdf_map/resolution_sdf", resolution_sdf_, 0.2);
        node_.param("sdf_map/ceil_height", ceil_height_, 5.0);
//        node_.param("sdf_map/update_rate", update_rate_, 10.0);
        node_.param("sdf_map/update_range", update_range_, 5.0);
        node_.param("sdf_map/inflate", inflate_, 0.2);
//        node_.param("sdf_map/radius_ignore", radius_ignore_, 0.2);

        cout << "origin_: " << origin_.transpose() << endl;
        cout << "map size: " << map_size_.transpose() << endl;
        cout << "resolution: " << resolution_sdf_ << endl;

        /* ---------- sub and pub ---------- */
        odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/prometheus/drone_odom", 10, &SDFMap_Global::odomCallback, this);

        global_point_clound_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl", 1, &SDFMap_Global::globalcloudCallback, this);
        
        inflate_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/inflate_cloud", 1);
        
        boundary = Eigen::VectorXd::Zero(6);
        ROS_INFO("global_point_sdf: sub pub finished!");
        /* ---------- setting ---------- */
        have_odom_ = false;
        map_valid_ = false;

        has_global_point = false;

        resolution_inv_ = 1 / resolution_sdf_;

        for (int i = 0; i < 3; ++i)
            grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
        min_range_ = origin_;
        max_range_ = origin_ + map_size_;

        //---------------------create a map using sdf_tools-----------------------------
        // sdf collision map parameter
        Eigen::Translation3d origin_translation(origin_(0), origin_(1), origin_(2));
        // Eigen::Translation3d origin_translation(-map_size_(0)/2, -map_size_(1)/2, 0.0);

        Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
        const Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
        const std::string frame = "world";  //  frame
        // create map
        // ROS_INFO("global_point_sdf: creat std_tools!");
        sdf_tools::COLLISION_CELL oob_cell;
        oob_cell.occupancy = 0.0;
        oob_cell.component = 0;
        // const sdf_tools::COLLISION_CELL oob_cell = cell;
        shared_ptr<sdf_tools::CollisionMapGrid> trans_sdf(new sdf_tools::CollisionMapGrid(origin_transform, frame, resolution_sdf_, map_size_(0), map_size_(1),
                                                   map_size_(2), oob_cell));
        collision_map = trans_sdf;
        ROS_INFO("global_point_sdf: init sdf_tools finished!");
    }

    void
    SDFMap_Global::getInterpolationData(const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &pos_vec, Eigen::Vector3d &diff) {
        if (!isInMap(pos)) {
            // cout << "pos invalid for interpolation." << endl;
        }

        /* interpolation position */
        Eigen::Vector3d pos_m = pos - 0.5 * resolution_sdf_ * Eigen::Vector3d::Ones();

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        diff = (pos - idx_pos) * resolution_inv_;

        pos_vec.clear();

        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++) {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    Eigen::Vector3d current_pos;
                    indexToPos(current_idx, current_pos);
                    pos_vec.push_back(current_pos);
                }
    }

}  // namespace dyn_planner
