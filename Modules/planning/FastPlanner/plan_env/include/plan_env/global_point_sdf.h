//
// Created by taojiang on 2020/1/8.
//

#ifndef PLANNING_GLOBAL_POINT_SDF_H
#define PLANNING_GLOBAL_POINT_SDF_H


#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


// sdf_tools
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/sdf.hpp"


using namespace std;

namespace dyn_planner
{

void constrains(double &n, double min, double max);

class SDFMap_Global{

private:
    // data are saved in vector

    // map property
    Eigen::Vector3d min_range_, max_range_;  // map range in pos
    Eigen::Vector3i grid_size_;              // map range in index

    bool isInMap(Eigen::Vector3d pos);
    void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
    void indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos);


    /* ---------- parameter ---------- */
    double inflate_, update_range_, radius_ignore_;
    Eigen::Vector3d origin_, map_size_;
    double resolution_sdf_, resolution_inv_;
    double ceil_height_, up_height_;
//    double update_rate_;

    /** virtual boundary, 6x1 vector, for min x max x... min z,max z */
    mutable Eigen::VectorXd boundary;

    //  sdf_tools
//    sdf_tools ::CollisionMapGrid collision_map
    /* ---------- callback ---------- */
    nav_msgs::Odometry odom_;
    bool have_odom_;

    pcl::PointCloud<pcl::PointXYZ> cloud_inflate_vis_;
    bool new_map_, map_valid_;

    bool has_global_point;
    pcl::PointCloud<pcl::PointXYZ> latest_global_cloud_;
    shared_ptr<sdf_tools ::CollisionMapGrid> collision_map;
    shared_ptr<sdf_tools::SignedDistanceField> sdf_map;

    ros::NodeHandle node_;
    ros::Subscriber odom_sub_, cloud_sub_, octomap_sub_;
    ros::Subscriber global_point_clound_sub_;
    ros::Publisher inflate_cloud_pub_;

    void globalcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    /* --------------------------------- */

public:
    SDFMap_Global() {}
    SDFMap_Global(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
    ~SDFMap_Global() {}
    void init(ros::NodeHandle& nh);

    /* get state */
    bool odomValid() { return have_odom_; }
    bool mapValid() { return map_valid_; }
    nav_msgs::Odometry getOdom() { return odom_; }
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) { ori = origin_, size = map_size_; }
    double getResolution() { return resolution_sdf_; }

    void getInterpolationData(const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& pos_vec,
                              Eigen::Vector3d& diff);

    // occupancy management
//    void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
//    void setOccupancy(Eigen::Vector3d pos, int occ = 1);
//    int getOccupancy(Eigen::Vector3d pos);
//    int getOccupancy(Eigen::Vector3i id);
//    void getOccupancyMarker(visualization_msgs::Marker& m, int id, Eigen::Vector4d color);

    // distance field management
    double getDistance(Eigen::Vector3d pos);
//    double getDistance(Eigen::Vector3i id);
    double evaluateEDTWithGrad(const Eigen::Vector3d& pos, double& dist, Eigen::Vector3d& grad);

//    double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
//    double getDistTrilinear(Eigen::Vector3d pos);
//    void setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos);
//    void updateESDF3d(bool neg = false);
//    void getESDFMarker(vector<visualization_msgs::Marker>& markers, int id, Eigen::Vector3d color);
//    double getMaxDistance();

    typedef shared_ptr<SDFMap_Global> Ptr;
};


}

#endif //PLANNING_GLOBAL_POINT_SDF_H



