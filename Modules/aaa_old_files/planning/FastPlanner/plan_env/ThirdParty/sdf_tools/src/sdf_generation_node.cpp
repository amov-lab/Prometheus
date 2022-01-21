#include <ros/ros.h>
#include "sdf_tools/SDF.h"
#include "sdf_tools/sdf_builder.hpp"
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <time.h>

visualization_msgs::Marker ExportCollisionMapForDisplay(VoxelGrid::VoxelGrid<uint8_t>& collision_map, std::string frame, float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame;
    // Populate the options
    display_rep.ns = "collision_map_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
    display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
    display_rep.scale.x = collision_map.GetCellSizes()[0];
    display_rep.scale.y = collision_map.GetCellSizes()[1];
    display_rep.scale.z = collision_map.GetCellSizes()[2];
    // Add all cells in collision
    for (int64_t x_index = 0; x_index < collision_map.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_map.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_map.GetNumZCells(); z_index++)
            {
                // Check if the current cell is in collision
                uint8_t status = collision_map.GetImmutable(x_index, y_index, z_index).first;
                if (status == 1)
                {
                    // Convert cell indices into a real-world location
                    std::vector<double> location = collision_map.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    // Color it
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 1.0;
                    display_rep.colors.push_back(new_color);
                }
                else
                {
                    assert(status == 0);
                }
            }
        }
    }
    return display_rep;
}

int main(int argc, char** argv)
{
    //test_voxel_grid();
    ros::init(argc, argv, "planning_scene_SDF_generator");
    ROS_INFO("Starting SDF from planning scene generator...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string frame;
    double resolution;
    double x_size;
    double y_size;
    double z_size;
    nhp.param(std::string("sdf_origin_frame"), frame, std::string("base"));
    nhp.param(std::string("resolution"), resolution, 0.05);
    nhp.param(std::string("x_size"), x_size, 1.0);
    nhp.param(std::string("y_size"), y_size, 2.0);
    nhp.param(std::string("z_size"), z_size, 1.0);
    Eigen::Translation3d origin_translation(0.1, -1.0, -0.3);
    Eigen::Quaterniond origin_rotation;
    origin_rotation.setIdentity();
    Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
    sdf_tools::SDF_Builder sdf_builder(nh, origin_transform, frame, x_size, y_size, z_size, resolution, INFINITY, "get_planning_scene");
    ROS_INFO("...startup complete");
    ////////////////////
    ///// Display! /////
    ////////////////////
    ros::Publisher viz_pub = nh.advertise<visualization_msgs::Marker>("sdf_markers", 1, true);
    ros::Rate spin_rate(10);
    while (ros::ok())
    {
        /* Collision map visualization message publish */
        std::cout << "Generating a new Collision Map..." << std::endl;
        VoxelGrid::VoxelGrid<uint8_t> coll_map = sdf_builder.UpdateCollisionMap(sdf_tools::USE_FULL_PLANNING_SCENE);
        std::cout << "...Collision Map with " << (coll_map.GetNumXCells() * coll_map.GetNumYCells() * coll_map.GetNumZCells()) << " cells generated - sending to RVIZ" << std::endl;
        viz_pub.publish(ExportCollisionMapForDisplay(coll_map, "base", 1.0));
        /* SDF visualization message publish */
        clock_t st, et;
        st = std::clock();
        sdf_tools::SignedDistanceField sdf = sdf_builder.UpdateSDF(sdf_tools::USE_CACHED);
        et = std::clock();
        std::cout << "SDF with " << (sdf.GetNumXCells() * sdf.GetNumYCells() * sdf.GetNumZCells()) << " cells generated - took " << (((float)(et - st)) / CLOCKS_PER_SEC) << " seconds to compute" << std::endl;
        viz_pub.publish(sdf.ExportForDisplay(0.1));
        ros::spinOnce();
        spin_rate.sleep();
    }
    return 0;
}
