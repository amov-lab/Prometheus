#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/dynamic_spatial_hashed_voxel_grid.hpp>
#include <sdf_tools/collision_map.hpp>
#include <sdf_tools/dynamic_spatial_hashed_collision_map.hpp>

namespace sdf_tools
{
    DynamicSpatialHashedCollisionMapGrid::DynamicSpatialHashedCollisionMapGrid(std::string frame, double resolution, int64_t chunk_x_size, int64_t chunk_y_size, int64_t chunk_z_size, COLLISION_CELL OOB_value)
    {
        frame_ = frame;
        VoxelGrid::DynamicSpatialHashedVoxelGrid<COLLISION_CELL> new_field(resolution, chunk_x_size, chunk_y_size, chunk_z_size, OOB_value);
        collision_field_ = new_field;
        number_of_components_ = 0;
        components_valid_ = false;
    }

    DynamicSpatialHashedCollisionMapGrid::DynamicSpatialHashedCollisionMapGrid(Eigen::Isometry3d origin_transform, std::string frame, double resolution, int64_t chunk_x_size, int64_t chunk_y_size, int64_t chunk_z_size, COLLISION_CELL OOB_value)
    {
        frame_ = frame;
        VoxelGrid::DynamicSpatialHashedVoxelGrid<COLLISION_CELL> new_field(origin_transform, resolution, chunk_x_size, chunk_y_size, chunk_z_size, OOB_value);
        collision_field_ = new_field;
        number_of_components_ = 0;
        components_valid_ = false;
    }

    DynamicSpatialHashedCollisionMapGrid::DynamicSpatialHashedCollisionMapGrid() : number_of_components_(0), initialized_(false), components_valid_(false) {}

    bool DynamicSpatialHashedCollisionMapGrid::IsInitialized() const
    {
        return initialized_;
    }

    bool DynamicSpatialHashedCollisionMapGrid::AreComponentsValid() const
    {
        return components_valid_;
    }

    std::pair<COLLISION_CELL, VoxelGrid::FOUND_STATUS> DynamicSpatialHashedCollisionMapGrid::Get(const double x, const double y, const double z) const
    {
        return collision_field_.GetImmutable(x, y, z);
    }

    std::pair<COLLISION_CELL, VoxelGrid::FOUND_STATUS> DynamicSpatialHashedCollisionMapGrid::Get(const Eigen::Vector3d& location) const
    {
        return collision_field_.GetImmutable(location);
    }

    VoxelGrid::SET_STATUS DynamicSpatialHashedCollisionMapGrid::SetCell(const double x, const double y, const double z, COLLISION_CELL value)
    {
        return collision_field_.SetCellValue(x, y, z, value);
    }

    VoxelGrid::SET_STATUS DynamicSpatialHashedCollisionMapGrid::SetCell(const Eigen::Vector3d& location, COLLISION_CELL value)
    {
        return collision_field_.SetCellValue(location, value);
    }

    VoxelGrid::SET_STATUS DynamicSpatialHashedCollisionMapGrid::SetChunk(const double x, const double y, const double z, COLLISION_CELL value)
    {
        return collision_field_.SetChunkValue(x, y, z, value);
    }

    VoxelGrid::SET_STATUS DynamicSpatialHashedCollisionMapGrid::SetChunk(const Eigen::Vector3d& location, COLLISION_CELL value)
    {
        return collision_field_.SetChunkValue(location, value);
    }

    Eigen::Isometry3d DynamicSpatialHashedCollisionMapGrid::GetOriginTransform() const
    {
        return collision_field_.GetOriginTransform();
    }

    std::vector<visualization_msgs::Marker> DynamicSpatialHashedCollisionMapGrid::ExportForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const
    {
        // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
        // We make one marker for the individual cells, and another for the chunks, but we return 0, 1, or 2
        // markers depending on if any chenks or cells can be drawn.
        // First, the chunks
        visualization_msgs::Marker chunks_display_rep;
        // Populate the header
        chunks_display_rep.header.frame_id = frame_;
        // Populate the options
        chunks_display_rep.ns = "dynamic_spatial_hashed_collision_map_chunks_display";
        chunks_display_rep.id = 1;
        chunks_display_rep.type = visualization_msgs::Marker::CUBE_LIST;
        chunks_display_rep.action = visualization_msgs::Marker::ADD;
        chunks_display_rep.lifetime = ros::Duration(0.0);
        chunks_display_rep.frame_locked = false;
        const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
        chunks_display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
        std::vector<double> chunk_sizes = collision_field_.GetChunkSizes();
        chunks_display_rep.scale.x = chunk_sizes[0];
        chunks_display_rep.scale.y = chunk_sizes[1];
        chunks_display_rep.scale.z = chunk_sizes[2];
        // Second, the cells
        visualization_msgs::Marker cells_display_rep;
        // Populate the header
        cells_display_rep.header.frame_id = frame_;
        // Populate the options
        cells_display_rep.ns = "dynamic_spatial_hashed_collision_map_cells_display";
        cells_display_rep.id = 1;
        cells_display_rep.type = visualization_msgs::Marker::CUBE_LIST;
        cells_display_rep.action = visualization_msgs::Marker::ADD;
        cells_display_rep.lifetime = ros::Duration(0.0);
        cells_display_rep.frame_locked = false;
        cells_display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
        std::vector<double> cell_sizes = collision_field_.GetCellSizes();
        cells_display_rep.scale.x = cell_sizes[0];
        cells_display_rep.scale.y = cell_sizes[1];
        cells_display_rep.scale.z = cell_sizes[2];
        // Now, go through the chunks and add everything to the message
        const Eigen::Isometry3d& grid_transform = GetOriginTransform();
        const std::unordered_map<VoxelGrid::CHUNK_REGION, VoxelGrid::DynamicSpatialHashedVoxelGridChunk<COLLISION_CELL>>& raw_chunks = collision_field_.GetInternalChunks();
        std::unordered_map<VoxelGrid::CHUNK_REGION, VoxelGrid::DynamicSpatialHashedVoxelGridChunk<COLLISION_CELL>>::const_iterator raw_chunks_itr;
        for (raw_chunks_itr = raw_chunks.begin(); raw_chunks_itr != raw_chunks.end(); ++raw_chunks_itr)
        {
            const VoxelGrid::DynamicSpatialHashedVoxelGridChunk<COLLISION_CELL>& current_chunk = raw_chunks_itr->second;
            if (current_chunk.IsChunkInitialized())
            {
                const COLLISION_CELL& current_cell = current_chunk.GetChunkImmutable();
                std::vector<double> cell_location_in_grid = current_chunk.GetIndexLocationInGrid(0, 0, 0);
                Eigen::Vector3d grid_location(cell_location_in_grid[0], cell_location_in_grid[1], cell_location_in_grid[2]);
                Eigen::Vector3d location = grid_transform * grid_location;
                geometry_msgs::Point new_point;
                new_point.x = location.x();
                new_point.y = location.y();
                new_point.z = location.z();
                chunks_display_rep.points.push_back(new_point);
                if (current_cell.occupancy > 0.5)
                {
                    chunks_display_rep.colors.push_back(collision_color);
                }
                else if (current_cell.occupancy < 0.5)
                {
                    chunks_display_rep.colors.push_back(free_color);
                }
                else
                {
                    chunks_display_rep.colors.push_back(unknown_color);
                }
            }
            else if (current_chunk.IsCellInitialized())
            {
                for (int64_t x_index = 0; x_index < current_chunk.GetNumXCells(); x_index++)
                {
                    for (int64_t y_index = 0; y_index < current_chunk.GetNumYCells(); y_index++)
                    {
                        for (int64_t z_index = 0; z_index < current_chunk.GetNumZCells(); z_index++)
                        {
                            const COLLISION_CELL& current_cell = current_chunk.GetImmutableByIndex(x_index, y_index, z_index).first;
                            std::vector<double> cell_location_in_grid = current_chunk.GetIndexLocationInGrid(x_index, y_index, z_index);
                            Eigen::Vector3d grid_location(cell_location_in_grid[0], cell_location_in_grid[1], cell_location_in_grid[2]);
                            Eigen::Vector3d location = grid_transform * grid_location;
                            geometry_msgs::Point new_point;
                            new_point.x = location.x();
                            new_point.y = location.y();
                            new_point.z = location.z();
                            cells_display_rep.points.push_back(new_point);
                            if (current_cell.occupancy > 0.5)
                            {
                                cells_display_rep.colors.push_back(collision_color);
                            }
                            else if (current_cell.occupancy < 0.5)
                            {
                                cells_display_rep.colors.push_back(free_color);
                            }
                            else
                            {
                                cells_display_rep.colors.push_back(unknown_color);
                            }
                        }
                    }
                }
            }
        }
        // Assemble the data to return
        std::vector<visualization_msgs::Marker> display_markers;
        if (chunks_display_rep.points.size() > 0)
        {
            display_markers.push_back(chunks_display_rep);
        }
        if (cells_display_rep.points.size() > 0)
        {
            display_markers.push_back(cells_display_rep);
        }
        return display_markers;
    }
}
