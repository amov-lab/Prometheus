#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <urdf_model/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/SDF.h"

#ifndef SDF_BUILDER_HPP
#define SDF_BUILDER_HPP

namespace sdf_tools
{
    static const uint8_t USE_CACHED = 0x00;
    static const uint8_t USE_ONLY_OCTOMAP = 0x01;
    static const uint8_t USE_ONLY_COLLISION_OBJECTS = 0x02;
    static const uint8_t USE_FULL_PLANNING_SCENE = 0x03;

    typedef struct
    {
        uint32_t location[3];
        uint32_t closest_point[3];
        double distance_square;
        int32_t update_direction;
    } bucket_cell;

    typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

    double ComputeDistanceSquared(int32_t x1, int32_t y1, int32_t z1, int32_t x2, int32_t y2, int32_t z2);

    class SDF_Builder
    {
    protected:

        bool initialized_;
        bool has_cached_sdf_;
        bool has_cached_collmap_;
        bool has_planning_scene_;
        Eigen::Isometry3d origin_transform_;
        std::string frame_;
        double x_size_;
        double y_size_;
        double z_size_;
        double resolution_;
        float OOB_value_;
        SignedDistanceField cached_sdf_;
        VoxelGrid::VoxelGrid<uint8_t> cached_collmap_;
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_ptr_;
        ros::NodeHandle nh_;
        ros::ServiceClient planning_scene_client_;

        SignedDistanceField UpdateSDFFromPlanningScene();

        VoxelGrid::VoxelGrid<uint8_t> UpdateCollisionMapFromPlanningScene();

        bool BuildInternalPlanningScene();

        DistanceField BuildDistanceField(std::vector<Eigen::Vector3i>& points);

        std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods();

        static int GetDirectionNumber(int dx, int dy, int dz);

        std::string GenerateSDFComputeBotURDFString();

        std::string GenerateSDFComputeBotSRDFString();

    public:

        SDF_Builder(ros::NodeHandle& nh, Eigen::Isometry3d origin_transform, std::string frame, double x_size, double y_size, double z_size, double resolution, float OOB_value, std::string planning_scene_service);

        SDF_Builder(ros::NodeHandle& nh, std::string frame, double x_size, double y_size, double z_size, double resolution, float OOB_value, std::string planning_scene_service);

        SDF_Builder();

        void UpdatePlanningSceneFromMessage(moveit_msgs::PlanningScene& planning_scene);

        SignedDistanceField UpdateSDF(uint8_t update_mode);

        SignedDistanceField GetCachedSDF();

        VoxelGrid::VoxelGrid<uint8_t> UpdateCollisionMap(uint8_t update_mode);

        VoxelGrid::VoxelGrid<uint8_t> GetCachedCollisionMap();

    };


}

#endif // SDF_BUILDER_HPP
