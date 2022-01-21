#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/dynamic_spatial_hashed_voxel_grid.hpp>
#include <sdf_tools/collision_map.hpp>

#ifndef DYNAMIC_SPATIAL_HASHED_COLLISION_MAP_HPP
#define DYNAMIC_SPATIAL_HASHED_COLLISION_MAP_HPP

namespace sdf_tools
{
    class DynamicSpatialHashedCollisionMapGrid
    {
    protected:

        VoxelGrid::DynamicSpatialHashedVoxelGrid<COLLISION_CELL> collision_field_;
        uint32_t number_of_components_;
        std::string frame_;
        bool initialized_;
        bool components_valid_;

    public:

        DynamicSpatialHashedCollisionMapGrid(std::string frame, double resolution, int64_t chunk_x_size, int64_t chunk_y_size, int64_t chunk_z_size, COLLISION_CELL OOB_value);

        DynamicSpatialHashedCollisionMapGrid(Eigen::Isometry3d origin_transform, std::string frame, double resolution, int64_t chunk_x_size, int64_t chunk_y_size, int64_t chunk_z_size, COLLISION_CELL OOB_value);

        DynamicSpatialHashedCollisionMapGrid();

        bool IsInitialized() const;

        bool AreComponentsValid() const;

        std::pair<COLLISION_CELL, VoxelGrid::FOUND_STATUS> Get(const double x, const double y, const double z) const;

        std::pair<COLLISION_CELL, VoxelGrid::FOUND_STATUS> Get(const Eigen::Vector3d& location) const;

        VoxelGrid::SET_STATUS SetCell(const double x, const double y, const double z, COLLISION_CELL value);

        VoxelGrid::SET_STATUS SetCell(const Eigen::Vector3d& location, COLLISION_CELL value);

        VoxelGrid::SET_STATUS SetChunk(const double x, const double y, const double z, COLLISION_CELL value);

        VoxelGrid::SET_STATUS SetChunk(const Eigen::Vector3d& location, COLLISION_CELL value);

        Eigen::Isometry3d GetOriginTransform() const;

        std::vector<visualization_msgs::Marker> ExportForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;
    };
}

#endif // DYNAMIC_SPATIAL_HASHED_COLLISION_MAP_HPP
