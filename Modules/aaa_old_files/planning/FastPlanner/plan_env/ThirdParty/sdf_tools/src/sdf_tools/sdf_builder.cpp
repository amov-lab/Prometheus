#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include <octomap_msgs/conversions.h>
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/sdf_builder.hpp"
#include "sdf_tools/SDF.h"

namespace sdf_tools
{
    double ComputeDistanceSquared(int32_t x1, int32_t y1, int32_t z1, int32_t x2, int32_t y2, int32_t z2)
    {
        int32_t dx = x1 - x2;
        int32_t dy = y1 - y2;
        int32_t dz = z1 - z2;
        return double((dx * dx) + (dy * dy) + (dz * dz));
    }

    SDF_Builder::SDF_Builder(ros::NodeHandle& nh, Eigen::Isometry3d origin_transform, std::string frame, double x_size, double y_size, double z_size, double resolution, float OOB_value, std::string planning_scene_service) : nh_(nh)
    {
        origin_transform_ = origin_transform;
        frame_ = frame;
        x_size_ = x_size;
        y_size_ = y_size;
        z_size_ = z_size;
        resolution_ = resolution;
        OOB_value_ = OOB_value;
        if (!BuildInternalPlanningScene())
        {
            throw std::invalid_argument("Unable to construct internal planning scene");
        }
        initialized_ = true;
        has_cached_sdf_ = false;
        has_cached_collmap_ = false;
        has_planning_scene_ = false;
        planning_scene_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_service);
    }

    SDF_Builder::SDF_Builder(ros::NodeHandle& nh, std::string frame, double x_size, double y_size, double z_size, double resolution, float OOB_value, std::string planning_scene_service) : nh_(nh)
    {
        Eigen::Translation3d origin_translation(-x_size * 0.5, -y_size * 0.5, -z_size * 0.5);
        Eigen::Quaterniond origin_rotation;
        origin_rotation.setIdentity();
        origin_transform_ = origin_translation * origin_rotation;
        frame_ = frame;
        x_size_ = x_size;
        y_size_ = y_size;
        z_size_ = z_size;
        resolution_ = resolution;
        OOB_value_ = OOB_value;
        if (!BuildInternalPlanningScene())
        {
            throw std::invalid_argument("Unable to construct internal planning scene");
        }
        initialized_ = true;
        has_cached_sdf_ = false;
        has_cached_collmap_ = false;
        has_planning_scene_ = false;
        planning_scene_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_service);
    }

    SDF_Builder::SDF_Builder()
    {
        initialized_ = false;
        has_cached_sdf_ = false;
        has_cached_collmap_ = false;
        has_planning_scene_ = false;
    }

    bool SDF_Builder::BuildInternalPlanningScene()
    {
        /* Builds a planning scene from XML string urdf and srdf descriptions */
        // Make the URDF model
        boost::shared_ptr<urdf::Model> urdf_model(new urdf::Model());
        urdf_model->initString(GenerateSDFComputeBotURDFString());
        // Make the SRDF model
        boost::shared_ptr<srdf::Model> srdf_model(new srdf::Model());
        srdf_model->initString(*urdf_model, GenerateSDFComputeBotSRDFString());
        // Make the planning scene
        planning_scene_ptr_.reset();
        planning_scene_ptr_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(urdf_model, srdf_model));
        return true;
    }

    std::string SDF_Builder::GenerateSDFComputeBotURDFString()
    {
        // Figure out the minimum+maximum X,Y,Z values (for safety, we pad them out to make sure)
        double min_x_limit = origin_transform_.translation().x() - fabs(2 * x_size_);
        double max_x_limit = origin_transform_.translation().x() + fabs(2 * x_size_);
        double min_y_limit = origin_transform_.translation().y() - fabs(2 * y_size_);
        double max_y_limit = origin_transform_.translation().y() + fabs(2 * y_size_);
        double min_z_limit = origin_transform_.translation().z() - fabs(2 * z_size_);
        double max_z_limit = origin_transform_.translation().z() + fabs(2 * z_size_);
        // Make the limits into strings
        std::ostringstream mnxls_strm;
        mnxls_strm << min_x_limit;
        std::string min_x_limit_str = mnxls_strm.str();
        std::ostringstream mxxls_strm;
        mxxls_strm << max_x_limit;
        std::string max_x_limit_str = mxxls_strm.str();
        std::ostringstream mnyls_strm;
        mnyls_strm << min_y_limit;
        std::string min_y_limit_str = mnyls_strm.str();
        std::ostringstream mxyls_strm;
        mxyls_strm << max_y_limit;
        std::string max_y_limit_str = mxyls_strm.str();
        std::ostringstream mnzls_strm;
        mnzls_strm << min_z_limit;
        std::string min_z_limit_str = mnzls_strm.str();
        std::ostringstream mxzls_strm;
        mxzls_strm << max_z_limit;
        std::string max_z_limit_str = mxzls_strm.str();
        // Figure out the cell resolution
        std::ostringstream crs_strm;
        crs_strm << resolution_;
        std::string cell_resolution_str = crs_strm.str();
        // Make the URDF xml string
        std::string urdf_string;
        urdf_string = "<?xml version=\"1.0\" ?>\n<robot name=\"sdf_compute_bot\">\n<link name=\"" + frame_ + "\">\n</link>\n<joint name=\"virtual_x\" type=\"prismatic\">\n<parent link=\"" + frame_ + "\"/>\n<child link=\"x_stage\"/>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<axis xyz=\"1 0 0\"/>\n<limit effort=\"10.0\" lower=\"" + min_x_limit_str + "\" upper=\"10.0\" velocity=\"" + max_x_limit_str + "\"/>\n</joint>\n<link name=\"x_stage\">\n</link>\n<joint name=\"virtual_y\" type=\"prismatic\">\n<parent link=\"x_stage\"/>\n<child link=\"y_stage\"/>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<axis xyz=\"0 1 0\"/>\n<limit effort=\"10.0\" lower=\"" + min_y_limit_str + "\" upper=\"" + max_y_limit_str + "\" velocity=\"10.0\"/>\n</joint>\n<link name=\"y_stage\">\n</link>\n<joint name=\"virtual_z\" type=\"prismatic\">\n<parent link=\"y_stage\"/>\n<child link=\"sdf_bot\"/>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<axis xyz=\"0 0 1\"/>\n<limit effort=\"10.0\" lower=\"" + min_z_limit_str + "\" upper=\"" + max_z_limit_str + "\" velocity=\"10.0\"/>\n</joint>\n<link name=\"sdf_bot\">\n<visual>\n<geometry>\n<box size=\"" + cell_resolution_str + " " + cell_resolution_str + " " + cell_resolution_str + "\"/>\n</geometry>\n<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n<material name=\"\">\n<color rgba=\"0.3 0.3 0.3 1.0\"/>\n</material>\n</visual>\n</link>\n</robot>";
        return urdf_string;
    }

    std::string SDF_Builder::GenerateSDFComputeBotSRDFString()
    {
        std::string srdf_string;
        srdf_string = "<?xml version=\"1.0\" ?>\n<robot name=\"sdf_compute_bot\">\n<group name=\"platform\">\n<joint name=\"virtual_x\" />\n<joint name=\"virtual_y\" />\n<joint name=\"virtual_z\" />\n</group>\n<disable_collisions link1=\"" + frame_ + "\" link2=\"x_stage\" reason=\"Adjacent\" />\n<disable_collisions link1=\"" + frame_ + "\" link2=\"y_stage\" reason=\"Adjacent\" />\n<disable_collisions link1=\"" + frame_ + "\" link2=\"sdf_bot\" reason=\"Adjacent\" />\n<disable_collisions link1=\"x_stage\" link2=\"y_stage\" reason=\"Adjacent\" />\n<disable_collisions link1=\"x_stage\" link2=\"sdf_bot\" reason=\"Adjacent\" />\n<disable_collisions link1=\"y_stage\" link2=\"sdf_bot\" reason=\"Adjacent\" />\n</robot>";
        return srdf_string;
    }

    SignedDistanceField SDF_Builder::UpdateSDF(uint8_t update_mode)
    {
        if (!initialized_)
        {
            throw std::invalid_argument("SDF Builder has not been initialized");
        }
        if (update_mode == USE_CACHED)
        {
            if (has_planning_scene_)
            {
                // Build the SDF
                return UpdateSDFFromPlanningScene();
            }
            else
            {
                ROS_ERROR("No planning scene available");
                throw std::invalid_argument("No planning scene available");
            }
        }
        else
        {
            if (update_mode == USE_ONLY_COLLISION_OBJECTS)
            {
                // Update the planning scene
                moveit_msgs::GetPlanningSceneRequest ps_req;
                ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
                moveit_msgs::GetPlanningSceneResponse ps_res;
                planning_scene_client_.call(ps_req, ps_res);
                moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
                planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
                has_planning_scene_ = true;
                // Build the SDF
                return UpdateSDFFromPlanningScene();
            }
            else if (update_mode == USE_ONLY_OCTOMAP)
            {
                // Update the planning scene
                moveit_msgs::GetPlanningSceneRequest ps_req;
                ps_req.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
                moveit_msgs::GetPlanningSceneResponse ps_res;
                planning_scene_client_.call(ps_req, ps_res);
                moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
                planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
                has_planning_scene_ = true;
                // Build the SDF
                return UpdateSDFFromPlanningScene();
            }
            else if (update_mode == USE_FULL_PLANNING_SCENE)
            {
                // Update the planning scene
                moveit_msgs::GetPlanningSceneRequest ps_req;
                ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP;
                moveit_msgs::GetPlanningSceneResponse ps_res;
                planning_scene_client_.call(ps_req, ps_res);
                moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
                planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
                has_planning_scene_ = true;
                // Build the SDF
                return UpdateSDFFromPlanningScene();
            }
            else
            {
                ROS_ERROR("Invalid update mode (mode not recognized)");
                throw std::invalid_argument("Invalid update mode (mode not recognized)");
            }
        }
    }

    SignedDistanceField SDF_Builder::GetCachedSDF()
    {
        if (has_cached_sdf_)
        {
            return cached_sdf_;
        }
        else
        {
            ROS_ERROR("No cached SDF available");
            throw std::invalid_argument("No cached SDF available");
        }
    }

    VoxelGrid::VoxelGrid<uint8_t> SDF_Builder::UpdateCollisionMap(uint8_t update_mode)
    {
        if (!initialized_)
        {
            throw std::invalid_argument("SDF Builder has not been initialized");
        }
        if (update_mode == USE_CACHED)
        {
            if (has_planning_scene_)
            {
                // Build the collision map
                return UpdateCollisionMapFromPlanningScene();
            }
            else
            {
                ROS_ERROR("No planning scene available");
                throw std::invalid_argument("No planning scene available");
            }
        }
        else
        {
            if (update_mode == USE_ONLY_COLLISION_OBJECTS)
            {
                // Update the planning scene
                moveit_msgs::GetPlanningSceneRequest ps_req;
                ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
                moveit_msgs::GetPlanningSceneResponse ps_res;
                planning_scene_client_.call(ps_req, ps_res);
                moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
                planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
                has_planning_scene_ = true;
                // Build the collision map
                return UpdateCollisionMapFromPlanningScene();
            }
            else if (update_mode == USE_ONLY_OCTOMAP)
            {
                // Update the planning scene
                moveit_msgs::GetPlanningSceneRequest ps_req;
                ps_req.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
                moveit_msgs::GetPlanningSceneResponse ps_res;
                planning_scene_client_.call(ps_req, ps_res);
                moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
                planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
                has_planning_scene_ = true;
                // Build the collision map
                return UpdateCollisionMapFromPlanningScene();
            }
            else if (update_mode == USE_FULL_PLANNING_SCENE)
            {
                // Update the planning scene
                moveit_msgs::GetPlanningSceneRequest ps_req;
                ps_req.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP;
                moveit_msgs::GetPlanningSceneResponse ps_res;
                planning_scene_client_.call(ps_req, ps_res);
                moveit_msgs::PlanningScene& planning_scene_state = ps_res.scene;
                planning_scene_ptr_->usePlanningSceneMsg(planning_scene_state);
                has_planning_scene_ = true;
                // Build the collision map
                return UpdateCollisionMapFromPlanningScene();
            }
            else
            {
                ROS_ERROR("Invalid update mode (mode not recognized)");
                throw std::invalid_argument("Invalid update mode (mode not recognized)");
            }
        }
    }

    VoxelGrid::VoxelGrid<uint8_t> SDF_Builder::GetCachedCollisionMap()
    {
        if (has_cached_collmap_)
        {
            return cached_collmap_;
        }
        else
        {
            ROS_ERROR("No cached Collision Map available");
            throw std::invalid_argument("No cached Collision Map available");
        }
    }

    VoxelGrid::VoxelGrid<uint8_t> SDF_Builder::UpdateCollisionMapFromPlanningScene()
    {
        // Make a collision field for debugging
        VoxelGrid::VoxelGrid<uint8_t> collision_field(origin_transform_, resolution_, x_size_, y_size_, z_size_, 0);
        // Loop through the planning scene to populate the voxel grids
        std::string x_joint("virtual_x");
        std::string y_joint("virtual_y");
        std::string z_joint("virtual_z");
        collision_detection::CollisionRequest col_req;
        collision_detection::CollisionResult col_res;
        robot_state::RobotState& sdf_compute_bot_state = planning_scene_ptr_->getCurrentStateNonConst();
        for (int64_t x_index = 0; x_index < collision_field.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < collision_field.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < collision_field.GetNumZCells(); z_index++)
                {
                    // Convert SDF indices into a real-world location
                    std::vector<double> location = collision_field.GridIndexToLocation(x_index, y_index, z_index);
                    double x = location[0];
                    double y = location[1];
                    double z = location[2];
                    // Set them
                    sdf_compute_bot_state.setJointPositions(x_joint, &x);
                    sdf_compute_bot_state.setJointPositions(y_joint, &y);
                    sdf_compute_bot_state.setJointPositions(z_joint, &z);
                    col_res.clear();
                    planning_scene_ptr_->checkCollision(col_req, col_res);
                    if (col_res.collision)
                    {
                        // Mark as filled
                        //std::cout << "Collision" << std::endl;
                        uint8_t status = 1;
                        collision_field.SetValue(x_index, y_index, z_index, status);
                    }
                    else
                    {
                        // Mark as free space
                        //std::cout << "No collision" << std::endl;
                        uint8_t status = 0;
                        collision_field.SetValue(x_index, y_index, z_index, status);
                    }
                }
            }
        }
        // Export the collision map
        cached_collmap_ = collision_field;
        has_cached_collmap_ = true;
        return collision_field;
    }

    SignedDistanceField SDF_Builder::UpdateSDFFromPlanningScene()
    {
        // Make the SDF
        SignedDistanceField new_sdf(origin_transform_, frame_, resolution_, x_size_, y_size_, z_size_, OOB_value_);
        // Loop through the planning scene to populate the voxel grids
        std::vector<Eigen::Vector3i> filled;
        std::vector<Eigen::Vector3i> free;
        std::string x_joint("virtual_x");
        std::string y_joint("virtual_y");
        std::string z_joint("virtual_z");
        collision_detection::CollisionRequest col_req;
        collision_detection::CollisionResult col_res;
        robot_state::RobotState& sdf_compute_bot_state = planning_scene_ptr_->getCurrentStateNonConst();
        for (uint32_t x_index = 0; x_index < new_sdf.GetNumXCells(); x_index++)
        {
            for (uint32_t y_index = 0; y_index < new_sdf.GetNumYCells(); y_index++)
            {
                for (uint32_t z_index = 0; z_index < new_sdf.GetNumZCells(); z_index++)
                {
                    // Convert SDF indices into a real-world location
                    std::vector<double> location = new_sdf.GridIndexToLocation(x_index, y_index, z_index);
                    double x = location[0];
                    double y = location[1];
                    double z = location[2];
                    sdf_compute_bot_state.setJointPositions(x_joint, &x);
                    sdf_compute_bot_state.setJointPositions(y_joint, &y);
                    sdf_compute_bot_state.setJointPositions(z_joint, &z);
                    col_res.clear();
                    planning_scene_ptr_->checkCollision(col_req, col_res);
                    if (col_res.collision)
                    {
                        // Mark as filled
                        filled.push_back(Eigen::Vector3i(x_index, y_index, z_index));
                    }
                    else
                    {
                        // Mark as free space
                        free.push_back(Eigen::Vector3i(x_index, y_index, z_index));
                    }
                }
            }
        }
        // Make two distance fields (one for distance to filled voxels, one for distance to free voxels
        DistanceField filled_distance_field = BuildDistanceField(filled);
        DistanceField free_distance_field = BuildDistanceField(free);
        // Generate the SDF
        for (int64_t x_index = 0; x_index < filled_distance_field.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < filled_distance_field.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < filled_distance_field.GetNumZCells(); z_index++)
                {
                    double distance1 = sqrt(filled_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * resolution_;
                    double distance2 = sqrt(free_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * resolution_;
                    new_sdf.Set(x_index, y_index, z_index, (distance1 - distance2));
                }
            }
        }
        // Export the SDF
        cached_sdf_ = new_sdf;
        has_cached_sdf_ = true;
        return new_sdf;
    }

    std::vector<std::vector<std::vector<std::vector<int>>>> SDF_Builder::MakeNeighborhoods()
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

    int SDF_Builder::GetDirectionNumber(int dx, int dy, int dz)
    {
        return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
    }

    DistanceField SDF_Builder::BuildDistanceField(std::vector<Eigen::Vector3i>& points)
    {
        // Make the DistanceField container
        bucket_cell default_cell;
        default_cell.distance_square = INFINITY;
        DistanceField distance_field(origin_transform_, resolution_, x_size_, y_size_, z_size_, default_cell);
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
            std::pair<bucket_cell&, bool> query = distance_field.GetMutable((int64_t)points[index].x(), (int64_t)points[index].y(), (int64_t)points[index].z());
            if (query.second)
            {
                query.first.location[0] = points[index].x();
                query.first.location[1] = points[index].y();
                query.first.location[2] = points[index].z();
                query.first.closest_point[0] = points[index].x();
                query.first.closest_point[1] = points[index].y();
                query.first.closest_point[2] = points[index].z();
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

    void SDF_Builder::UpdatePlanningSceneFromMessage(moveit_msgs::PlanningScene& planning_scene)
    {
        planning_scene_ptr_->usePlanningSceneMsg(planning_scene);
        has_planning_scene_ = true;
    }
}
