#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include <ros/ros.h>
#include <list>
#include <unordered_map>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <sdf_tools/collision_map.hpp>
#include <sdf_tools/CollisionMap.h>

namespace sdf_tools
{
    COLLISION_CELL::COLLISION_CELL(const float in_occupancy, const uint32_t in_component) : occupancy(in_occupancy), component(in_component) {}

    std::vector<uint8_t> CollisionCellToBinary(const COLLISION_CELL& value)
    {
        std::vector<uint8_t> binary(sizeof(COLLISION_CELL));
        memcpy(&binary.front(), &value, sizeof(COLLISION_CELL));
        return binary;
    }

    COLLISION_CELL CollisionCellFromBinary(const std::vector<uint8_t>& binary)
    {
        if (binary.size() != sizeof(COLLISION_CELL))
        {
            std::cerr << "Binary value is not " << sizeof(COLLISION_CELL) << " bytes" << std::endl;
            return COLLISION_CELL(NAN, 0u);
        }
        else
        {
            COLLISION_CELL loaded;
            memcpy(&loaded, &binary.front(), sizeof(COLLISION_CELL));
            return loaded;
        }
    }

    std_msgs::ColorRGBA CollisionMapGrid::GenerateComponentColor(const uint32_t component, const float alpha)
    {
        return arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(component, alpha);
    }

    bool CollisionMapGrid::IsSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        // First, we make sure that indices are within bounds
        // Out of bounds indices are NOT surface cells
        if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= GetNumXCells() || y_index >= GetNumYCells() || z_index >= GetNumZCells())
        {
            return false;
        }
        // Edge indices are automatically surface cells
        if (x_index == 0 || y_index == 0 || z_index == 0 || x_index == (GetNumXCells() - 1) || y_index == (GetNumYCells() - 1) || z_index == (GetNumZCells()))
        {
            return true;
        }
        // If the cell is inside the grid, we check the neighbors
        // Note that we must check all 26 neighbors
        uint32_t our_component = collision_field_.GetImmutable(x_index, y_index, z_index).first.component;
        // Check neighbor 1
        if (our_component != collision_field_.GetImmutable(x_index, y_index, z_index - 1).first.component)
        {
            return true;
        }
        // Check neighbor 2
        else if (our_component != collision_field_.GetImmutable(x_index, y_index, z_index + 1).first.component)
        {
            return true;
        }
        // Check neighbor 3
        else if (our_component != collision_field_.GetImmutable(x_index, y_index - 1, z_index).first.component)
        {
            return true;
        }
        // Check neighbor 4
        else if (our_component != collision_field_.GetImmutable(x_index, y_index + 1, z_index).first.component)
        {
            return true;
        }
        // Check neighbor 5
        else if (our_component != collision_field_.GetImmutable(x_index - 1, y_index, z_index).first.component)
        {
            return true;
        }
        // Check neighbor 6
        else if (our_component != collision_field_.GetImmutable(x_index + 1, y_index, z_index).first.component)
        {
            return true;
        }
        // If none of the faces are exposed, it's not a surface voxel
        return false;
    }

    CollisionMapGrid::DistanceField CollisionMapGrid::BuildDistanceField(const std::vector<VoxelGrid::GRID_INDEX>& points) const
    {
        // Make the DistanceField container
        bucket_cell default_cell;
        default_cell.distance_square = INFINITY;
        DistanceField distance_field(collision_field_.GetOriginTransform(), GetResolution(), collision_field_.GetXSize(), collision_field_.GetYSize(), collision_field_.GetZSize(), default_cell);
        // Compute maximum distance square
        long max_distance_square = (distance_field.GetNumXCells() * distance_field.GetNumXCells()) + (distance_field.GetNumYCells() * distance_field.GetNumYCells()) + (distance_field.GetNumZCells() * distance_field.GetNumZCells());
        // Make bucket queue
        std::vector<std::vector<bucket_cell>> bucket_queue(max_distance_square + 1);
        bucket_queue[0].reserve(points.size());
        // Set initial update direction
        int initial_update_direction = GetDirectionNumber(0, 0, 0);
        // Mark all points with distance zero and add to the bucket queue
        for (size_t index = 0; index < points.size(); index++)
        {
            const VoxelGrid::GRID_INDEX& current_index = points[index];
            std::pair<bucket_cell&, bool> query = distance_field.GetMutable(current_index);
            if (query.second)
            {
                query.first.location[0] = current_index.x;
                query.first.location[1] = current_index.y;
                query.first.location[2] = current_index.z;
                query.first.closest_point[0] = current_index.x;
                query.first.closest_point[1] = current_index.y;
                query.first.closest_point[2] = current_index.z;
                query.first.distance_square = 0.0;
                query.first.update_direction = initial_update_direction;
                bucket_queue[0].push_back(query.first);
            }
            // If the point is outside the bounds of the SDF, skip
            else
            {
                continue;
            }
        }
        // Process the bucket queue
        std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods = MakeNeighborhoods();
        for (size_t bq_idx = 0; bq_idx < bucket_queue.size(); bq_idx++)
        {
            std::vector<bucket_cell>::iterator queue_itr = bucket_queue[bq_idx].begin();
            while (queue_itr != bucket_queue[bq_idx].end())
            {
                // Get the current location
                bucket_cell& cur_cell = *queue_itr;
                double x = cur_cell.location[0];
                double y = cur_cell.location[1];
                double z = cur_cell.location[2];
                // Pick the update direction
                int D = bq_idx;
                if (D > 1)
                {
                    D = 1;
                }
                // Make sure the update direction is valid
                if (cur_cell.update_direction < 0 || cur_cell.update_direction > 26)
                {
                    ++queue_itr;
                    continue;
                }
                // Get the current neighborhood list
                std::vector<std::vector<int>>& neighborhood = neighborhoods[D][cur_cell.update_direction];
                // Update the distance from the neighboring cells
                for (size_t nh_idx = 0; nh_idx < neighborhood.size(); nh_idx++)
                {
                    // Get the direction to check
                    int dx = neighborhood[nh_idx][0];
                    int dy = neighborhood[nh_idx][1];
                    int dz = neighborhood[nh_idx][2];
                    int nx = x + dx;
                    int ny = y + dy;
                    int nz = z + dz;
                    std::pair<bucket_cell&, bool> neighbor_query = distance_field.GetMutable((int64_t)nx, (int64_t)ny, (int64_t)nz);
                    if (!neighbor_query.second)
                    {
                        // "Neighbor" is outside the bounds of the SDF
                        continue;
                    }
                    // Update the neighbor's distance based on the current
                    int new_distance_square = ComputeDistanceSquared(nx, ny, nz, cur_cell.closest_point[0], cur_cell.closest_point[1], cur_cell.closest_point[2]);
                    if (new_distance_square > max_distance_square)
                    {
                        // Skip these cases
                        continue;
                    }
                    if (new_distance_square < neighbor_query.first.distance_square)
                    {
                        // If the distance is better, time to update the neighbor
                        neighbor_query.first.distance_square = new_distance_square;
                        neighbor_query.first.closest_point[0] = cur_cell.closest_point[0];
                        neighbor_query.first.closest_point[1] = cur_cell.closest_point[1];
                        neighbor_query.first.closest_point[2] = cur_cell.closest_point[2];
                        neighbor_query.first.location[0] = nx;
                        neighbor_query.first.location[1] = ny;
                        neighbor_query.first.location[2] = nz;
                        neighbor_query.first.update_direction = GetDirectionNumber(dx, dy, dz);
                        // Add the neighbor into the bucket queue
                        bucket_queue[new_distance_square].push_back(neighbor_query.first);
                    }
                }
                // Increment the queue iterator
                ++queue_itr;
            }
            // Clear the current queue now that we're done with it
            bucket_queue[bq_idx].clear();
        }
        return distance_field;
    }

    std::vector<std::vector<std::vector<std::vector<int>>>> CollisionMapGrid::MakeNeighborhoods() const
    {
        std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods;
        neighborhoods.resize(2);
        for (size_t n = 0; n < neighborhoods.size(); n++)
        {
            neighborhoods[n].resize(27);
            // Loop through the source directions
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        int direction_number = GetDirectionNumber(dx, dy, dz);
                        // Loop through the target directions
                        for (int tdx = -1; tdx <= 1; tdx++)
                        {
                            for (int tdy = -1; tdy <= 1; tdy++)
                            {
                                for (int tdz = -1; tdz <= 1; tdz++)
                                {
                                    if (tdx == 0 && tdy == 0 && tdz == 0)
                                    {
                                        continue;
                                    }
                                    if (n >= 1)
                                    {
                                        if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1)
                                        {
                                            continue;
                                        }
                                        if ((dx * tdx) < 0 || (dy * tdy) < 0 || (dz * tdz) < 0)
                                        {
                                            continue;
                                        }
                                    }
                                    std::vector<int> new_point;
                                    new_point.resize(3);
                                    new_point[0] = tdx;
                                    new_point[1] = tdy;
                                    new_point[2] = tdz;
                                    neighborhoods[n][direction_number].push_back(new_point);
                                }
                            }
                        }
                    }
                }
            }
        }
        return neighborhoods;
    }

    int CollisionMapGrid::GetDirectionNumber(const int dx, const int dy, const int dz) const
    {
        return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
    }

    double CollisionMapGrid::ComputeDistanceSquared(const int32_t x1, const int32_t y1, const int32_t z1, const int32_t x2, const int32_t y2, const int32_t z2) const
    {
        int32_t dx = x1 - x2;
        int32_t dy = y1 - y2;
        int32_t dz = z1 - z2;
        return double((dx * dx) + (dy * dy) + (dz * dz));
    }

    int64_t CollisionMapGrid::MarkConnectedComponent(int64_t x_index, int64_t y_index, int64_t z_index, uint32_t connected_component)
    {
        // Make the working queue
        std::list<VoxelGrid::GRID_INDEX> working_queue;
        // Make a hash table to store queued indices (so we don't repeat work)
        // Let's provide an hint at the size of hashmap we'll need, since this will reduce the need to resize & rehash
        // We're going to assume that connected components, in general, will take ~1/16 of the grid in size
        // which means, with 2 cells/hash bucket, we'll initialize to grid size/32
    #ifdef ENABLE_UNORDERED_MAP_SIZE_HINTS
        size_t queued_hashtable_size_hint = collision_field_.GetRawData().size() / 32;
        std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> queued_hashtable(queued_hashtable_size_hint);
    #else
        std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> queued_hashtable;
    #endif
        // Add the starting index
        VoxelGrid::GRID_INDEX start_index(x_index, y_index, z_index);
        // Enqueue it
        working_queue.push_back(start_index);
        queued_hashtable[start_index] = 1;
        // Work
        int64_t marked_cells = 0;
        while (working_queue.size() > 0)
        {
            // Get an item off the queue to work with
            VoxelGrid::GRID_INDEX current_index = working_queue.front();
            working_queue.pop_front();
            // Get the current value
            COLLISION_CELL current_value = collision_field_.GetImmutable(current_index.x, current_index.y, current_index.z).first;
            // Mark the connected component
            current_value.component = connected_component;
            // Update the grid
            collision_field_.SetValue(current_index.x, current_index.y, current_index.z, current_value);
            // Go through the possible neighbors and enqueue as needed
            // Since there are only six cases (voxels must share a face to be considered connected), we handle each explicitly
            // Case 1
            std::pair<COLLISION_CELL, bool> xm1_neighbor = collision_field_.GetImmutable(current_index.x - 1, current_index.y, current_index.z);
            if (xm1_neighbor.second && (current_value.occupancy == xm1_neighbor.first.occupancy))
            {
                VoxelGrid::GRID_INDEX neighbor_index(current_index.x - 1, current_index.y, current_index.z);
                if (queued_hashtable[neighbor_index] <= 0)
                {
                    queued_hashtable[neighbor_index] = 1;
                    working_queue.push_back(neighbor_index);
                }
            }
            // Case 2
            std::pair<COLLISION_CELL, bool> ym1_neighbor = collision_field_.GetImmutable(current_index.x, current_index.y - 1, current_index.z);
            if (ym1_neighbor.second && (current_value.occupancy == ym1_neighbor.first.occupancy))
            {
                VoxelGrid::GRID_INDEX neighbor_index(current_index.x, current_index.y - 1, current_index.z);
                if (queued_hashtable[neighbor_index] <= 0)
                {
                    queued_hashtable[neighbor_index] = 1;
                    working_queue.push_back(neighbor_index);
                }
            }
            // Case 3
            std::pair<COLLISION_CELL, bool> zm1_neighbor = collision_field_.GetImmutable(current_index.x, current_index.y, current_index.z - 1);
            if (zm1_neighbor.second && (current_value.occupancy == zm1_neighbor.first.occupancy))
            {
                VoxelGrid::GRID_INDEX neighbor_index(current_index.x, current_index.y, current_index.z - 1);
                if (queued_hashtable[neighbor_index] <= 0)
                {
                    queued_hashtable[neighbor_index] = 1;
                    working_queue.push_back(neighbor_index);
                }
            }
            // Case 4
            std::pair<COLLISION_CELL, bool> xp1_neighbor = collision_field_.GetImmutable(current_index.x + 1, current_index.y, current_index.z);
            if (xp1_neighbor.second && (current_value.occupancy == xp1_neighbor.first.occupancy))
            {
                VoxelGrid::GRID_INDEX neighbor_index(current_index.x + 1, current_index.y, current_index.z);
                if (queued_hashtable[neighbor_index] <= 0)
                {
                    queued_hashtable[neighbor_index] = 1;
                    working_queue.push_back(neighbor_index);
                }
            }
            // Case 5
            std::pair<COLLISION_CELL, bool> yp1_neighbor = collision_field_.GetImmutable(current_index.x, current_index.y + 1, current_index.z);
            if (yp1_neighbor.second && (current_value.occupancy == yp1_neighbor.first.occupancy))
            {
                VoxelGrid::GRID_INDEX neighbor_index(current_index.x, current_index.y + 1, current_index.z);
                if (queued_hashtable[neighbor_index] <= 0)
                {
                    queued_hashtable[neighbor_index] = 1;
                    working_queue.push_back(neighbor_index);
                }
            }
            // Case 6
            std::pair<COLLISION_CELL, bool> zp1_neighbor = collision_field_.GetImmutable(current_index.x, current_index.y, current_index.z + 1);
            if (zp1_neighbor.second && (current_value.occupancy == zp1_neighbor.first.occupancy))
            {
                VoxelGrid::GRID_INDEX neighbor_index(current_index.x, current_index.y, current_index.z + 1);
                if (queued_hashtable[neighbor_index] <= 0)
                {
                    queued_hashtable[neighbor_index] = 1;
                    working_queue.push_back(neighbor_index);
                }
            }
        }
        return marked_cells;
    }

    CollisionMapGrid::CollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const COLLISION_CELL& default_value, const COLLISION_CELL& OOB_value) : initialized_(true)
    {
        frame_ = frame;
        VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, default_value, OOB_value);
        collision_field_ = new_field;
        number_of_components_ = 0;
        components_valid_ = false;
    }

    CollisionMapGrid::CollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, double y_size, const double z_size, const COLLISION_CELL& default_value, const COLLISION_CELL& OOB_value) : initialized_(true)
    {
        frame_ = frame;
        VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, default_value, OOB_value);
        collision_field_ = new_field;
        number_of_components_ = 0;
        components_valid_ = false;
    }

    CollisionMapGrid::CollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const COLLISION_CELL& OOB_default_value) : initialized_(true)
    {
        frame_ = frame;
        VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(resolution, x_size, y_size, z_size, OOB_default_value);
        collision_field_ = new_field;
        number_of_components_ = 0;
        components_valid_ = false;
    }

    CollisionMapGrid::CollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, double y_size, const double z_size, const COLLISION_CELL& OOB_default_value) : initialized_(true)
    {
        frame_ = frame;
        VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_default_value);
        collision_field_ = new_field;
        number_of_components_ = 0;
        components_valid_ = false;
    }

    CollisionMapGrid::CollisionMapGrid() : number_of_components_(0), initialized_(false), components_valid_(false) {}

    bool CollisionMapGrid::IsInitialized() const
    {
        return initialized_;
    }

    bool CollisionMapGrid::AreComponentsValid() const
    {
        return components_valid_;
    }

    std::pair<COLLISION_CELL, bool> CollisionMapGrid::Get3d(const Eigen::Vector3d& location) const
    {
        return collision_field_.GetImmutable3d(location);
    }

    std::pair<COLLISION_CELL, bool> CollisionMapGrid::Get4d(const Eigen::Vector4d& location) const
    {
        return collision_field_.GetImmutable4d(location);
    }

    std::pair<COLLISION_CELL, bool> CollisionMapGrid::Get(const double x, const double y, const double z) const
    {
        return collision_field_.GetImmutable(x, y, z);
    }

    std::pair<COLLISION_CELL, bool> CollisionMapGrid::Get(const VoxelGrid::GRID_INDEX& index) const
    {
        return collision_field_.GetImmutable(index);
    }

    std::pair<COLLISION_CELL, bool> CollisionMapGrid::Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        return collision_field_.GetImmutable(x_index, y_index, z_index);
    }

    bool CollisionMapGrid::Set(const double x, const double y, const double z, COLLISION_CELL value)
    {
        components_valid_ = false;
        return collision_field_.SetValue(x, y, z, value);
    }

    bool CollisionMapGrid::Set3d(const Eigen::Vector3d& location, COLLISION_CELL value)
    {
        components_valid_ = false;
        return collision_field_.SetValue3d(location, value);
    }

    bool CollisionMapGrid::Set4d(const Eigen::Vector4d& location, COLLISION_CELL value)
    {
        components_valid_ = false;
        return collision_field_.SetValue4d(location, value);
    }

    bool CollisionMapGrid::Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, COLLISION_CELL value)
    {
        components_valid_ = false;
        return collision_field_.SetValue(x_index, y_index, z_index, value);
    }

    bool CollisionMapGrid::Set(const VoxelGrid::GRID_INDEX& index, COLLISION_CELL value)
    {
        components_valid_ = false;
        return collision_field_.SetValue(index, value);
    }

    double CollisionMapGrid::GetXSize() const
    {
        return collision_field_.GetXSize();
    }

    double CollisionMapGrid::GetYSize() const
    {
        return collision_field_.GetYSize();
    }

    double CollisionMapGrid::GetZSize() const
    {
        return collision_field_.GetZSize();
    }

    double CollisionMapGrid::GetResolution() const
    {
        return collision_field_.GetCellSizes()[0];
    }

    COLLISION_CELL CollisionMapGrid::GetDefaultValue() const
    {
        return collision_field_.GetDefaultValue();
    }

    COLLISION_CELL CollisionMapGrid::GetOOBValue() const
    {
        return collision_field_.GetOOBValue();
    }

    int64_t CollisionMapGrid::GetNumXCells() const
    {
        return collision_field_.GetNumXCells();
    }

    int64_t CollisionMapGrid::GetNumYCells() const
    {
        return collision_field_.GetNumYCells();
    }

    int64_t CollisionMapGrid::GetNumZCells() const
    {
        return collision_field_.GetNumZCells();
    }

    const Eigen::Isometry3d& CollisionMapGrid::GetOriginTransform() const
    {
        return collision_field_.GetOriginTransform();
    }

    const Eigen::Isometry3d& CollisionMapGrid::GetInverseOriginTransform() const
    {
        return collision_field_.GetInverseOriginTransform();
    }

    std::string CollisionMapGrid::GetFrame() const
    {
        return frame_;
    }

    std::pair<uint32_t, bool> CollisionMapGrid::GetNumConnectedComponents() const
    {
        return std::pair<uint32_t, bool>(number_of_components_, components_valid_);
    }

    std::vector<int64_t> CollisionMapGrid::LocationToGridIndex3d(const Eigen::Vector3d& location) const
    {
        return collision_field_.LocationToGridIndex3d(location);
    }

    std::vector<int64_t> CollisionMapGrid::LocationToGridIndex4d(const Eigen::Vector4d& location) const
    {
        return collision_field_.LocationToGridIndex4d(location);
    }

    std::vector<int64_t> CollisionMapGrid::LocationToGridIndex(double x, double y, double z) const
    {
        return collision_field_.LocationToGridIndex(x, y, z);
    }

    std::vector<double> CollisionMapGrid::GridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index) const
    {
        return collision_field_.GridIndexToLocation(x_index, y_index, z_index);
    }

    bool CollisionMapGrid::SaveToFile(const std::string &filepath)
    {
        // Convert to message representation
        sdf_tools::CollisionMap message_rep = GetMessageRepresentation();
        // Save message to file
        try
        {
            std::ofstream output_file(filepath.c_str(), std::ios::out|std::ios::binary);
            uint32_t serialized_size = ros::serialization::serializationLength(message_rep);
            std::unique_ptr<uint8_t> ser_buffer(new uint8_t[serialized_size]);
            ros::serialization::OStream ser_stream(ser_buffer.get(), serialized_size);
            ros::serialization::serialize(ser_stream, message_rep);
            output_file.write((char*)ser_buffer.get(), serialized_size);
            output_file.close();
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool CollisionMapGrid::LoadFromFile(const std::string& filepath)
    {
        try
        {
            // Load message from file
            std::ifstream input_file(filepath.c_str(), std::ios::in|std::ios::binary);
            input_file.seekg(0, std::ios::end);
            std::streampos end = input_file.tellg();
            input_file.seekg(0, std::ios::beg);
            std::streampos begin = input_file.tellg();
            uint32_t serialized_size = end - begin;
            std::unique_ptr<uint8_t> deser_buffer(new uint8_t[serialized_size]);
            input_file.read((char*) deser_buffer.get(), serialized_size);
            ros::serialization::IStream deser_stream(deser_buffer.get(), serialized_size);
            sdf_tools::CollisionMap new_message;
            ros::serialization::deserialize(deser_stream, new_message);
            // Load state from the message
            bool success = LoadFromMessageRepresentation(new_message);
            return success;
        }
        catch (...)
        {
            return false;
        }
    }

    sdf_tools::CollisionMap CollisionMapGrid::GetMessageRepresentation()
    {
        sdf_tools::CollisionMap message_rep;
        // Populate message
        message_rep.header.frame_id = frame_;
        const Eigen::Isometry3d& origin_transform = collision_field_.GetOriginTransform();
        message_rep.origin_transform.translation.x = origin_transform.translation().x();
        message_rep.origin_transform.translation.y = origin_transform.translation().y();
        message_rep.origin_transform.translation.z = origin_transform.translation().z();
        const Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
        message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
        message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
        message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
        message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
        message_rep.dimensions.x = GetXSize();
        message_rep.dimensions.y = GetYSize();
        message_rep.dimensions.z = GetZSize();
        message_rep.cell_size = GetResolution();
        message_rep.OOB_occupancy_value = collision_field_.GetDefaultValue().occupancy;
        message_rep.OOB_component_value = collision_field_.GetDefaultValue().component;
        message_rep.number_of_components = number_of_components_;
        message_rep.components_valid = components_valid_;
        message_rep.initialized = initialized_;
        std::vector<COLLISION_CELL> raw_data = collision_field_.GetRawData();
        std::vector<uint8_t> binary_data = PackBinaryRepresentation(raw_data);
        message_rep.data = ZlibHelpers::CompressBytes(binary_data);
        return message_rep;
    }

    bool CollisionMapGrid::LoadFromMessageRepresentation(sdf_tools::CollisionMap& message)
    {
        // Make a new voxel grid inside
        Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
        Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
        Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
        COLLISION_CELL OOB_value;
        OOB_value.occupancy = message.OOB_occupancy_value;
        OOB_value.component = message.OOB_component_value;
        VoxelGrid::VoxelGrid<COLLISION_CELL> new_field(origin_transform, message.cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, OOB_value);
        // Unpack the binary data
        std::vector<uint8_t> binary_representation = ZlibHelpers::DecompressBytes(message.data);
        std::vector<COLLISION_CELL> unpacked = UnpackBinaryRepresentation(binary_representation);
        if (unpacked.empty())
        {
            std::cerr << "Unpack returned an empty CollisionMapGrid" << std::endl;
            return false;
        }
        bool success = new_field.SetRawData(unpacked);
        if (!success)
        {
            std::cerr << "Unable to set internal representation of the CollisionMapGrid" << std::endl;
            return false;
        }
        // Set it
        collision_field_ = new_field;
        frame_ = message.header.frame_id;
        number_of_components_ = message.number_of_components;
        components_valid_ = message.components_valid;
        initialized_ = message.initialized;
        return true;
    }

    std::vector<uint8_t> CollisionMapGrid::PackBinaryRepresentation(std::vector<COLLISION_CELL>& raw)
    {
        std::vector<uint8_t> packed(raw.size() * 8);
        for (size_t field_idx = 0, binary_index = 0; field_idx < raw.size(); field_idx++, binary_index+=8)
        {
            COLLISION_CELL raw_cell = raw[field_idx];
            std::vector<uint8_t> packed_cell = CollisionCellToBinary(raw_cell);
            packed[binary_index] = packed_cell[0];
            packed[binary_index + 1] = packed_cell[1];
            packed[binary_index + 2] = packed_cell[2];
            packed[binary_index + 3] = packed_cell[3];
            packed[binary_index + 4] = packed_cell[4];
            packed[binary_index + 5] = packed_cell[5];
            packed[binary_index + 6] = packed_cell[6];
            packed[binary_index + 7] = packed_cell[7];
        }
        return packed;
    }

    std::vector<COLLISION_CELL> CollisionMapGrid::UnpackBinaryRepresentation(std::vector<uint8_t>& packed)
    {
        if ((packed.size() % 8) != 0)
        {
            std::cerr << "Invalid binary representation - length is not a multiple of 8" << std::endl;
            return std::vector<COLLISION_CELL>();
        }
        uint64_t data_size = packed.size() / 8;
        std::vector<COLLISION_CELL> unpacked(data_size);
        for (size_t field_idx = 0, binary_index = 0; field_idx < unpacked.size(); field_idx++, binary_index+=8)
        {
            std::vector<uint8_t> binary_block{packed[binary_index], packed[binary_index + 1], packed[binary_index + 2], packed[binary_index + 3], packed[binary_index + 4], packed[binary_index + 5], packed[binary_index + 6], packed[binary_index + 7]};
            unpacked[field_idx] = CollisionCellFromBinary(binary_block);
        }
        return unpacked;
    }

    uint32_t CollisionMapGrid::UpdateConnectedComponents()
    {
        // If the connected components are already valid, skip computing them again
        if (components_valid_)
        {
            return number_of_components_;
        }
        components_valid_ = false;
        // Reset components first
        for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
                {
                    COLLISION_CELL current = collision_field_.GetImmutable(x_index, y_index, z_index).first;
                    current.component = 0;
                    collision_field_.SetValue(x_index, y_index, z_index, current);
                }
            }
        }
        // Mark the components
        int64_t total_cells = collision_field_.GetNumXCells() * collision_field_.GetNumYCells() * collision_field_.GetNumZCells();
        int64_t marked_cells = 0;
        uint32_t connected_components = 0;
        // Sweep through the grid
        for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
                {
                    // Check if the cell has already been marked, if so, ignore
                    if (collision_field_.GetImmutable(x_index, y_index, z_index).first.component > 0)
                    {
                        continue;
                    }
                    // Start marking a new connected component
                    connected_components++;
                    int64_t cells_marked = MarkConnectedComponent(x_index, y_index, z_index, connected_components);
                    marked_cells += cells_marked;
                    // Short-circuit if we've marked everything
                    if (marked_cells == total_cells)
                    {
                        number_of_components_ = connected_components;
                        components_valid_ = true;
                        return connected_components;
                    }
                }
            }
        }
        number_of_components_ = connected_components;
        components_valid_ = true;
        return connected_components;
    }

    std::map<uint32_t, std::pair<int32_t, int32_t>> CollisionMapGrid::ComputeComponentTopology(bool ignore_empty_components, bool recompute_connected_components, bool verbose)
    {
        // Recompute the connected components if need be
        if (recompute_connected_components)
        {
            UpdateConnectedComponents();
        }
        // Extract the surfaces of each connected component
        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> component_surfaces = ExtractComponentSurfaces(ignore_empty_components);
        // Compute the number of holes in each surface
        std::map<uint32_t, std::pair<int32_t, int32_t>> component_holes;
        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>>::iterator component_surfaces_itr;
        for (component_surfaces_itr = component_surfaces.begin(); component_surfaces_itr != component_surfaces.end(); ++component_surfaces_itr)
        {
            uint32_t component_number = component_surfaces_itr->first;
            std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& component_surface = component_surfaces_itr->second;
            std::pair<int32_t, int32_t> number_of_holes_and_voids = ComputeHolesInSurface(component_number, component_surface, verbose);
            component_holes[component_number] = number_of_holes_and_voids;
        }
        return component_holes;
    }

    std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> CollisionMapGrid::ExtractComponentSurfaces(const bool ignore_empty_components) const
    {
        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> component_surfaces;
        // Loop through the grid and extract surface cells for each component
        for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
                {
                    COLLISION_CELL current_cell = collision_field_.GetImmutable(x_index, y_index, z_index).first;
                    if (ignore_empty_components)
                    {
                        if (current_cell.occupancy > 0.5)
                        {
                            VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                            if (IsSurfaceIndex(x_index, y_index, z_index))
                            {
                                component_surfaces[current_cell.component][current_index] = 1;
                            }
                        }
                    }
                    else
                    {
                        VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                        if (IsSurfaceIndex(x_index, y_index, z_index))
                        {
                            component_surfaces[current_cell.component][current_index] = 1;
                        }
                    }
                }
            }
        }
        return component_surfaces;
    }

    std::pair<int32_t, int32_t> CollisionMapGrid::ComputeHolesInSurface(const uint32_t component, const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const bool verbose) const
    {
        // We have a list of all voxels with an exposed surface face
        // We loop through this list of voxels, and convert each voxel
        // into 8 vertices (the corners), which we individually check:
        //
        // First - we check to see if the vertex has already been
        // evaluated
        //
        // Second - we check if the vertex is actually on the surface
        // (make sure at least one of the three adjacent vertices is
        // exposed)
        //
        // Third - insert into hashtable of surface vertices
        //
        // Once we have completed this process, we loop back through
        // the hashtable of surface vertices and compute the number
        // of distance-1 neighboring surface vertices (we do this by
        // checking each of the six potential neighbor vertices) and
        // keep a running count of all vertices with 3, 5, and 6
        // neighbors.
        //
        // Once we have evaluated all the neighbors of all surface
        // vertices, we count the number of holes in the grid using
        // the formula from Chen and Rong, "Linear Time Recognition
        // Algorithms for Topological Invariants in 3D":
        //
        // #holes = 1 + (M5 + 2 * M6 - M3) / 8
        //
        // where M5 is the number of vertices with 5 neighbors,
        // M6 is the number of vertices with 6 neighbors, and
        // M3 is the number of vertices with 3 neighbors
        //
        // Storage for surface vertices
        // Compute a hint for initial surface vertex hashmap size
        // expected # of surface vertices
        // surface cells * 8
    #ifdef ENABLE_UNORDERED_MAP_SIZE_HINTS
        size_t surface_vertices_size_hint = surface.size() * 8;
        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> surface_vertices(surface_vertices_size_hint);
    #else
        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> surface_vertices;
    #endif
        // Loop through all the surface voxels and extract surface vertices
        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>::const_iterator surface_itr;
        for (surface_itr = surface.begin(); surface_itr != surface.end(); ++surface_itr)
        {
            const VoxelGrid::GRID_INDEX& current_index = surface_itr->first;
            // First, grab all six neighbors from the grid
            std::pair<const COLLISION_CELL&, bool> xyzm1 = collision_field_.GetImmutable(current_index.x, current_index.y, current_index.z - 1);
            std::pair<const COLLISION_CELL&, bool> xyzp1 = collision_field_.GetImmutable(current_index.x, current_index.y, current_index.z + 1);
            std::pair<const COLLISION_CELL&, bool> xym1z = collision_field_.GetImmutable(current_index.x, current_index.y - 1, current_index.z);
            std::pair<const COLLISION_CELL&, bool> xyp1z = collision_field_.GetImmutable(current_index.x, current_index.y + 1, current_index.z);
            std::pair<const COLLISION_CELL&, bool> xm1yz = collision_field_.GetImmutable(current_index.x - 1, current_index.y, current_index.z);
            std::pair<const COLLISION_CELL&, bool> xp1yz = collision_field_.GetImmutable(current_index.x + 1, current_index.y, current_index.z);
            // Generate all 8 vertices for the current voxel, check if an adjacent vertex is on the surface, and insert it if so
            // First, check the (-,-,-) vertex
            if (component != xyzm1.first.component || component != xym1z.first.component || component != xm1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex1(current_index.x, current_index.y, current_index.z);
                surface_vertices[vertex1] = 1;
            }
            // Second, check the (-,-,+) vertex
            if (component != xyzp1.first.component || component != xym1z.first.component || component != xm1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex2(current_index.x, current_index.y, current_index.z + 1);
                surface_vertices[vertex2] = 1;
            }
            // Third, check the (-,+,-) vertex
            if (component != xyzm1.first.component || component != xyp1z.first.component || component != xm1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex3(current_index.x, current_index.y + 1, current_index.z);
                surface_vertices[vertex3] = 1;
            }
            // Fourth, check the (-,+,+) vertex
            if (component != xyzp1.first.component || component != xyp1z.first.component || component != xm1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex4(current_index.x, current_index.y + 1, current_index.z + 1);
                surface_vertices[vertex4] = 1;
            }
            // Fifth, check the (+,-,-) vertex
            if (component != xyzm1.first.component || component != xym1z.first.component || component != xp1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex5(current_index.x + 1, current_index.y, current_index.z);
                surface_vertices[vertex5] = 1;
            }
            // Sixth, check the (+,-,+) vertex
            if (component != xyzp1.first.component || component != xym1z.first.component || component != xp1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex6(current_index.x + 1, current_index.y, current_index.z + 1);
                surface_vertices[vertex6] = 1;
            }
            // Seventh, check the (+,+,-) vertex
            if (component != xyzm1.first.component || component != xyp1z.first.component || component != xp1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex7(current_index.x + 1, current_index.y + 1, current_index.z);
                surface_vertices[vertex7] = 1;
            }
            // Eighth, check the (+,+,+) vertex
            if (component != xyzp1.first.component || component != xyp1z.first.component || component != xp1yz.first.component)
            {
                VoxelGrid::GRID_INDEX vertex8(current_index.x + 1, current_index.y + 1, current_index.z + 1);
                surface_vertices[vertex8] = 1;
            }
        }
        if (verbose)
        {
            std::cerr << "Surface with " << surface.size() << " voxels has " << surface_vertices.size() << " surface vertices" << std::endl;
        }
        // Iterate through the surface vertices and count the neighbors of each vertex
        int32_t M3 = 0;
        int32_t M5 = 0;
        int32_t M6 = 0;
        // Store the connectivity of each vertex
        // Compute a hint for initial vertex connectivity hashmap size
        // real # of surface vertices
        // surface vertices
        size_t vertex_connectivity_size_hint = surface_vertices.size();
        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> vertex_connectivity(vertex_connectivity_size_hint);
        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>::iterator surface_vertices_itr;
        for (surface_vertices_itr = surface_vertices.begin(); surface_vertices_itr != surface_vertices.end(); ++surface_vertices_itr)
        {
            VoxelGrid::GRID_INDEX key = surface_vertices_itr->first;
            VoxelGrid::GRID_INDEX value = key;
            // Insert into the connectivity map
            vertex_connectivity[key] = 0b00000000;
            // Check the six edges from the current vertex and count the number of exposed edges
            // (an edge is exposed if the at least one of the four surrounding voxels is not part
            // of the current component)
            int32_t edge_count = 0;
            // First, get the 8 voxels that surround the current vertex
            std::pair<const COLLISION_CELL&, bool> xm1ym1zm1 = collision_field_.GetImmutable(value.x - 1, value.y - 1, value.z - 1);
            std::pair<const COLLISION_CELL&, bool> xm1ym1zp1 = collision_field_.GetImmutable(value.x - 1, value.y - 1, value.z + 0);
            std::pair<const COLLISION_CELL&, bool> xm1yp1zm1 = collision_field_.GetImmutable(value.x - 1, value.y + 0, value.z - 1);
            std::pair<const COLLISION_CELL&, bool> xm1yp1zp1 = collision_field_.GetImmutable(value.x - 1, value.y + 0, value.z + 0);
            std::pair<const COLLISION_CELL&, bool> xp1ym1zm1 = collision_field_.GetImmutable(value.x + 0, value.y - 1, value.z - 1);
            std::pair<const COLLISION_CELL&, bool> xp1ym1zp1 = collision_field_.GetImmutable(value.x + 0, value.y - 1, value.z + 0);
            std::pair<const COLLISION_CELL&, bool> xp1yp1zm1 = collision_field_.GetImmutable(value.x + 0, value.y + 0, value.z - 1);
            std::pair<const COLLISION_CELL&, bool> xp1yp1zp1 = collision_field_.GetImmutable(value.x + 0, value.y + 0, value.z + 0);
            // Check the "z- down" edge
            if (component != xm1ym1zm1.first.component || component != xm1yp1zm1.first.component || component != xp1ym1zm1.first.component || component != xp1yp1zm1.first.component)
            {
                if (!(component != xm1ym1zm1.first.component && component != xm1yp1zm1.first.component && component != xp1ym1zm1.first.component && component != xp1yp1zm1.first.component))
                {
                    edge_count++;
                    vertex_connectivity[key] |= 0b00000001;
                }
            }
            // Check the "z+ up" edge
            if (component != xm1ym1zp1.first.component || component != xm1yp1zp1.first.component || component != xp1ym1zp1.first.component || component != xp1yp1zp1.first.component)
            {
                if (!(component != xm1ym1zp1.first.component && component != xm1yp1zp1.first.component && component != xp1ym1zp1.first.component && component != xp1yp1zp1.first.component))
                {
                    edge_count++;
                    vertex_connectivity[key] |= 0b00000010;
                }
            }
            // Check the "y- right" edge
            if (component != xm1ym1zm1.first.component || component != xm1ym1zp1.first.component || component != xp1ym1zm1.first.component || component != xp1ym1zp1.first.component)
            {
                if (!(component != xm1ym1zm1.first.component && component != xm1ym1zp1.first.component && component != xp1ym1zm1.first.component && component != xp1ym1zp1.first.component))
                {
                    edge_count++;
                    vertex_connectivity[key] |= 0b00000100;
                }
            }
            // Check the "y+ left" edge
            if (component != xm1yp1zm1.first.component || component != xm1yp1zp1.first.component || component != xp1yp1zm1.first.component || component != xp1yp1zp1.first.component)
            {
                if (!(component != xm1yp1zm1.first.component && component != xm1yp1zp1.first.component && component != xp1yp1zm1.first.component && component != xp1yp1zp1.first.component))
                {
                    edge_count++;
                    vertex_connectivity[key] |= 0b00001000;
                }
            }
            // Check the "x- back" edge
            if (component != xm1ym1zm1.first.component || component != xm1ym1zp1.first.component || component != xm1yp1zm1.first.component || component != xm1yp1zp1.first.component)
            {
                if (!(component != xm1ym1zm1.first.component && component != xm1ym1zp1.first.component && component != xm1yp1zm1.first.component && component != xm1yp1zp1.first.component))
                {
                    edge_count++;
                    vertex_connectivity[key] |= 0b00010000;
                }
            }
            // Check the "x+ front" edge
            if (component != xp1ym1zm1.first.component || component != xp1ym1zp1.first.component || component != xp1yp1zm1.first.component || component != xp1yp1zp1.first.component)
            {
                if (!(component != xp1ym1zm1.first.component && component != xp1ym1zp1.first.component && component != xp1yp1zm1.first.component && component != xp1yp1zp1.first.component))
                {
                    edge_count++;
                    vertex_connectivity[key] |= 0b00100000;
                }
            }
            // Increment M counts
            if (edge_count == 3)
            {
                M3++;
            }
            else if (edge_count == 5)
            {
                M5++;
            }
            else if (edge_count == 6)
            {
                M6++;
            }
        }
        // Check to see if the set of vertices is connected. If not, our object contains void(s)
        int32_t number_of_surfaces = ComputeConnectivityOfSurfaceVertices(vertex_connectivity);
        int32_t number_of_voids = number_of_surfaces - 1;
        // Compute the number of holes in the surface
        int32_t raw_number_of_holes = 1 + ((M5 + (2 * M6) - M3) / 8);
        int32_t number_of_holes = raw_number_of_holes + number_of_voids;
        if (verbose)
        {
            std::cout << "Processing surface with M3 = " << M3 << " M5 = " << M5 << " M6 = " << M6 << " holes = " << number_of_holes << " surfaces = " << number_of_surfaces << " voids = " << number_of_voids << std::endl;
        }
        return std::pair<int32_t, int32_t>(number_of_holes, number_of_voids);
    }

    int32_t CollisionMapGrid::ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface_vertex_connectivity) const
    {
        int32_t connected_components = 0;
        int64_t processed_vertices = 0;
        // Compute a hint for initial vertex components hashmap size
        // real # of surface vertices
        // surface vertices
        size_t vertex_components_size_hint = surface_vertex_connectivity.size();
        std::unordered_map<VoxelGrid::GRID_INDEX, int32_t> vertex_components(vertex_components_size_hint);
        // Iterate through the vertices
        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>::const_iterator surface_vertices_itr;
        for (surface_vertices_itr = surface_vertex_connectivity.begin(); surface_vertices_itr != surface_vertex_connectivity.end(); ++surface_vertices_itr)
        {
            VoxelGrid::GRID_INDEX key = surface_vertices_itr->first;
            VoxelGrid::GRID_INDEX location = key;
            //const uint8_t& connectivity = surface_vertices_itr->second.second;
            // First, check if the vertex has already been marked
            if (vertex_components[key] > 0)
            {
                continue;
            }
            else
            {
                // If not, we start marking a new connected component
                connected_components++;
                // Make the working queue
                std::list<VoxelGrid::GRID_INDEX> working_queue;
                // Make a hash table to store queued indices (so we don't repeat work)
                // Compute a hint for initial queued hashtable hashmap size
                // If we assume that most object surfaces are, in fact, intact, then the first (and only)
                // queued_hashtable will need to store an entry for every vertex on the surface.
                // real # of surface vertices
                // surface vertices
                size_t queued_hashtable_size_hint = surface_vertex_connectivity.size();
                std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> queued_hashtable(queued_hashtable_size_hint);
                // Add the current point
                working_queue.push_back(location);
                queued_hashtable[key] = 1;
                // Keep track of the number of vertices we've processed
                int64_t component_processed_vertices = 0;
                // Loop from the queue
                while (working_queue.size() > 0)
                {
                    // Get the top of thw working queue
                    VoxelGrid::GRID_INDEX current_vertex = working_queue.front();
                    working_queue.pop_front();
                    component_processed_vertices++;
                    vertex_components[current_vertex] = connected_components;
                    // Check the six possibly-connected vertices and add them to the queue if they are connected
                    // Get the connectivity of our index
                    uint8_t connectivity = surface_vertex_connectivity.at(current_vertex);
                    // Go through the neighbors
                    if ((connectivity & 0b00000001) > 0)
                    {
                        // Try to add the vertex
                        VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y, current_vertex.z - 1);
                        // We only add if we haven't already processed it
                        if (queued_hashtable[connected_vertex] <= 0)
                        {
                            queued_hashtable[connected_vertex] = 1;
                            working_queue.push_back(connected_vertex);
                        }
                    }
                    if ((connectivity & 0b00000010) > 0)
                    {
                        // Try to add the vertex
                        VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y, current_vertex.z + 1);
                        // We only add if we haven't already processed it
                        if (queued_hashtable[connected_vertex] <= 0)
                        {
                            queued_hashtable[connected_vertex] = 1;
                            working_queue.push_back(connected_vertex);
                        }
                    }
                    if ((connectivity & 0b00000100) > 0)
                    {
                        // Try to add the vertex
                        VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y - 1, current_vertex.z);
                        // We only add if we haven't already processed it
                        if (queued_hashtable[connected_vertex] <= 0)
                        {
                            queued_hashtable[connected_vertex] = 1;
                            working_queue.push_back(connected_vertex);
                        }
                    }
                    if ((connectivity & 0b00001000) > 0)
                    {
                        // Try to add the vertex
                        VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y + 1, current_vertex.z);
                        // We only add if we haven't already processed it
                        if (queued_hashtable[connected_vertex] <= 0)
                        {
                            queued_hashtable[connected_vertex] = 1;
                            working_queue.push_back(connected_vertex);
                        }
                    }
                    if ((connectivity & 0b00010000) > 0)
                    {
                        // Try to add the vertex
                        VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x - 1, current_vertex.y, current_vertex.z);
                        // We only add if we haven't already processed it
                        if (queued_hashtable[connected_vertex] <= 0)
                        {
                            queued_hashtable[connected_vertex] = 1;
                            working_queue.push_back(connected_vertex);
                        }
                    }
                    if ((connectivity & 0b00100000) > 0)
                    {
                        // Try to add the vertex
                        VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x + 1, current_vertex.y, current_vertex.z);
                        // We only add if we haven't already processed it
                        if (queued_hashtable[connected_vertex] <= 0)
                        {
                            queued_hashtable[connected_vertex] = 1;
                            working_queue.push_back(connected_vertex);
                        }
                    }
                }
                processed_vertices += component_processed_vertices;
                if (processed_vertices == (int64_t)surface_vertex_connectivity.size())
                {
                    break;
                }
            }
        }
        return connected_components;
    }

    std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> CollisionMapGrid::ExtractSignedDistanceField(const float oob_value) const

    {
        // Make the SDF
        SignedDistanceField new_sdf(collision_field_.GetOriginTransform(), frame_, GetResolution(), collision_field_.GetXSize(), collision_field_.GetYSize(), collision_field_.GetZSize(), oob_value);
        std::vector<VoxelGrid::GRID_INDEX> filled;
        std::vector<VoxelGrid::GRID_INDEX> free;
        for (int64_t x_index = 0; x_index < new_sdf.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < new_sdf.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < new_sdf.GetNumZCells(); z_index++)
                {
                    VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                    COLLISION_CELL stored = Get(x_index, y_index, z_index).first;
                    if (stored.occupancy > 0.5)
                    {
                        // Mark as filled
                        filled.push_back(current_index);
                    }
                    else
                    {
                        // Mark as free space
                        free.push_back(current_index);
                    }
                }
            }
        }
        // Make two distance fields (one for distance to filled voxels, one for distance to free voxels
        DistanceField filled_distance_field = BuildDistanceField(filled);
        DistanceField free_distance_field = BuildDistanceField(free);
        // Generate the SDF
        double max_distance = -INFINITY;
        double min_distance = INFINITY;
        for (int64_t x_index = 0; x_index < filled_distance_field.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < filled_distance_field.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < filled_distance_field.GetNumZCells(); z_index++)
                {
                    double distance1 = sqrt(filled_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * new_sdf.GetResolution();
                    double distance2 = sqrt(free_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * new_sdf.GetResolution();
                    double distance = distance1 - distance2;
                    if (distance > max_distance)
                    {
                        max_distance = distance;
                    }
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                    new_sdf.Set(x_index, y_index, z_index, distance);
                }
            }
        }
        std::pair<double, double> extrema(max_distance, min_distance);
        return std::pair<SignedDistanceField, std::pair<double, double>>(new_sdf, extrema);
    }

    visualization_msgs::Marker CollisionMapGrid::ExportForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const
    {
        // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
        visualization_msgs::Marker display_rep;
        // Populate the header
        display_rep.header.frame_id = frame_;
        // Populate the options
        display_rep.ns = "collision_map_display";
        display_rep.id = 1;
        display_rep.type = visualization_msgs::Marker::CUBE_LIST;
        display_rep.action = visualization_msgs::Marker::ADD;
        display_rep.lifetime = ros::Duration(0.0);
        display_rep.frame_locked = false;
        const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
        display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
        display_rep.scale.x = GetResolution();
        display_rep.scale.y = GetResolution();
        display_rep.scale.z = GetResolution();
        // Add all the cells of the SDF to the message
        for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
                {
                    // Convert grid indices into a real-world location
                    std::vector<double> location = collision_field_.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    if (collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
                    {
                        if (collision_color.a > 0.0)
                        {
                            display_rep.points.push_back(new_point);
                            display_rep.colors.push_back(collision_color);
                        }
                    }
                    else if (collision_field_.GetImmutable(x_index, y_index, z_index).first.occupancy < 0.5)
                    {
                        if (free_color.a > 0.0)
                        {
                            display_rep.points.push_back(new_point);
                            display_rep.colors.push_back(free_color);
                        }
                    }
                    else
                    {
                        if (unknown_color.a > 0.0)
                        {
                            display_rep.points.push_back(new_point);
                            display_rep.colors.push_back(unknown_color);
                        }
                    }
                }
            }
        }
        return display_rep;
    }

    visualization_msgs::Marker CollisionMapGrid::ExportConnectedComponentsForDisplay(bool color_unknown_components) const
    {
        // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
        visualization_msgs::Marker display_rep;
        // Populate the header
        display_rep.header.frame_id = frame_;
        // Populate the options
        display_rep.ns = "connected_components_display";
        display_rep.id = 1;
        display_rep.type = visualization_msgs::Marker::CUBE_LIST;
        display_rep.action = visualization_msgs::Marker::ADD;
        display_rep.lifetime = ros::Duration(0.0);
        display_rep.frame_locked = false;
        const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
        display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
        display_rep.scale.x = GetResolution();
        display_rep.scale.y = GetResolution();
        display_rep.scale.z = GetResolution();
        // Add all the cells of the SDF to the message
        for (int64_t x_index = 0; x_index < collision_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < collision_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < collision_field_.GetNumZCells(); z_index++)
                {
                    // Convert grid indices into a real-world location
                    std::vector<double> location = collision_field_.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    COLLISION_CELL current_cell = collision_field_.GetImmutable(x_index, y_index, z_index).first;
                    if (current_cell.occupancy != 0.5)
                    {
                        std_msgs::ColorRGBA color = GenerateComponentColor(current_cell.component);
                        display_rep.colors.push_back(color);
                    }
                    else
                    {
                        if (color_unknown_components)
                        {
                            std_msgs::ColorRGBA color = GenerateComponentColor(current_cell.component);
                            display_rep.colors.push_back(color);
                        }
                        else
                        {
                            std_msgs::ColorRGBA color;
                            color.a = 1.0;
                            color.r = 0.5;
                            color.g = 0.5;
                            color.b = 0.5;
                            display_rep.colors.push_back(color);
                        }
                    }
                }
            }
        }
        return display_rep;
    }


}
