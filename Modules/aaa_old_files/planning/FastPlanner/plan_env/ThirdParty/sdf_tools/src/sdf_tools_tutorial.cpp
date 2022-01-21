#include <ros/ros.h>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/pretty_print.hpp>
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/CollisionMap.h"
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/SDF.h"

int main(int argc, char** argv)
{
    // Make a ROS node, which we'll use to publish copies of the data in the CollisionMap and SDF
    // and Rviz markers that allow us to visualize them.
    ros::init(argc, argv, "sdf_tools_tutorial");
    // Get a handle to the current node
    ros::NodeHandle nh;
    // Make a publisher for visualization messages
    ros::Publisher visualization_pub = nh.advertise<visualization_msgs::Marker>("sdf_tools_tutorial_visualization", 1, true);
    // Make a publisher for serialized CollisionMaps
    ros::Publisher collision_map_pub = nh.advertise<sdf_tools::CollisionMap>("collision_map_pub", 1, true);
    // Make a publisher for serialized SDFs
    ros::Publisher sdf_pub = nh.advertise<sdf_tools::SDF>("sdf_pub", 1, true);
    // In preparation, we want to set a couple common paramters
    double resolution = 0.25;
    double x_size = 10.0;
    double y_size = 10.0;
    double z_size = 10.0;
    // Let's center the grid around the origin
    Eigen::Translation3d origin_translation(-5.0, -5.0, -5.0);
    Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
    Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
    std::string frame = "tutorial_frame";
    ///////////////////////////////////
    //// Let's make a CollisionMap ////
    ///////////////////////////////////
    // We pick a reasonable out-of-bounds value
    sdf_tools::COLLISION_CELL oob_cell;
    oob_cell.occupancy = 0.0;
    oob_cell.component = 0; // This should ALWAYS be zero, unless you know exactly what you're doing
    // Instead, we could initialize it like this - the component value is automatically set to 0
    sdf_tools::COLLISION_CELL oob_cell_2(0.0);
    // First, let's make the container
    sdf_tools::CollisionMapGrid collision_map(origin_transform, frame, resolution, x_size, y_size, z_size, oob_cell);
    // Let's set some values
    // This is how you should iterate through the 3D grid's cells
    for (int64_t x_index = 0; x_index < collision_map.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < collision_map.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < collision_map.GetNumZCells(); z_index++)
            {
                // Let's make the bottom corner (low x, low y, low z) an object
                if ((x_index < (collision_map.GetNumXCells() / 2)) && (y_index < (collision_map.GetNumYCells() / 2)) && (z_index < (collision_map.GetNumZCells() / 2)))
                {
                    sdf_tools::COLLISION_CELL obstacle_cell(1.0); // Occupancy values > 0.5 are obstacles
                    collision_map.Set(x_index, y_index, z_index, obstacle_cell);
                }
            }
        }
    }
    // We can also set by location
    sdf_tools::COLLISION_CELL obstacle_cell(1.0); // Occupancy values > 0.5 are obstacles
    collision_map.Set(0.0, 0.0, 0.0, obstacle_cell);
    // Let's get some values
    // We can query by index
    int64_t x_index = 10;
    int64_t y_index = 10;
    int64_t z_index = 10;
    std::pair<sdf_tools::COLLISION_CELL, bool> index_query = collision_map.Get(x_index, y_index, z_index);
    std::cout << "Index query result - stored value " << index_query.first.occupancy << " (occupancy) " << index_query.first.component << " (component) was it in the grid? - " << index_query.second << std::endl;
    // Or we can query by location
    double x_location = 0.0;
    double y_location = 0.0;
    double z_location = 0.0;
    std::pair<sdf_tools::COLLISION_CELL, bool> location_query = collision_map.Get(x_location, y_location, z_location);
    std::cout << "Location query result - stored value " << location_query.first.occupancy << " (occupancy) " << location_query.first.component << " (component) was it in the grid? - " << location_query.second << std::endl;
    // Let's compute connected components
    uint32_t num_connected_components = collision_map.UpdateConnectedComponents();
    std::cout << " There are " << num_connected_components << " connected components in the grid" << std::endl;
    // Let's display the results to Rviz
    // First, the CollisionMap itself
    // We need to provide colors to use
    std_msgs::ColorRGBA collision_color;
    collision_color.r = 1.0;
    collision_color.g = 0.0;
    collision_color.b = 0.0;
    collision_color.a = 0.5;
    std_msgs::ColorRGBA free_color;
    free_color.r = 0.0;
    free_color.g = 1.0;
    free_color.b = 0.0;
    free_color.a = 0.5;
    std_msgs::ColorRGBA unknown_color;
    unknown_color.r = 1.0;
    unknown_color.g = 1.0;
    unknown_color.b = 0.0;
    unknown_color.a = 0.5;
    visualization_msgs::Marker collision_map_marker = collision_map.ExportForDisplay(collision_color, free_color, unknown_color);
    // To be safe, you'll need to set these yourself. The namespace (ns) value should distinguish between different things being displayed
    // while the id value lets you have multiple versions of the same message at once. Always set this to 1 if you only want one copy.
    collision_map_marker.ns = "collision_map";
    collision_map_marker.id = 1;
    // Send it off for display
    visualization_pub.publish(collision_map_marker);
    // Now, let's draw the connected components
    visualization_msgs::Marker connected_components_marker = collision_map.ExportConnectedComponentsForDisplay(false); // Generally, you don't want a special color for unknown [P(occupancy) = 0.5] components
    connected_components_marker.ns = "connected_components";
    connected_components_marker.id = 1;
    visualization_pub.publish(connected_components_marker);
    // Let's export the CollisionMap - this is how you can transfer it to another ROS node
    collision_map_pub.publish(collision_map.GetMessageRepresentation());
    // You can also save it to a file
    std::string collision_map_filename = "collision_map.cmg";
    collision_map.SaveToFile(collision_map_filename);
    // And load it back in
    bool loaded = collision_map.LoadFromFile(collision_map_filename);
    if (loaded)
    {
        std::cout << "Reloaded CollisionMap from file" << std::endl;
    }
    else
    {
        std::cerr << "Whoa, something broke!" << std::endl;
    }
    ///////////////////////////
    //// Let's make an SDF ////
    ///////////////////////////
    // We pick a reasonable out-of-bounds value
    float oob_value = INFINITY;
    // We start by extracting the SDF from the CollisionMap
    std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf_with_extrema = collision_map.ExtractSignedDistanceField(oob_value);
    sdf_tools::SignedDistanceField& sdf = sdf_with_extrema.first;
    std::pair<double, double> sdf_extrema = sdf_with_extrema.second;
    std::cout << "Maximum distance in the SDF: " << sdf_extrema.first << ", minimum distance in the SDF: " << sdf_extrema.second << std::endl;
    // We lock the SDF to prevent unintended changes that would invalidate it
    sdf.Lock();
    // Let's get some values
    std::pair<float, bool> index_sdf_query = sdf.GetSafe(x_index, y_index, z_index);
    std::cout << "Index query result - stored distance " << index_sdf_query.first << " was it in the grid? - " << index_sdf_query.second << std::endl;
    std::pair<float, bool> location_sdf_query = sdf.GetSafe(x_location, y_location, z_location);
    std::cout << "Location query result - stored distance " << location_sdf_query.first << " was it in the grid? - " << location_sdf_query.second << std::endl;
    // Let's get some gradients
    std::vector<double> index_gradient_query = sdf.GetGradient(x_index, y_index, z_index, true); // Usually, you want to enable 'edge gradients' i.e. gradients for cells on the edge of the grid that don't have 6 neighbors
    std::cout << "Index gradient query result - gradient " << PrettyPrint::PrettyPrint(index_gradient_query) << std::endl;
    std::vector<double> location_gradient_query = sdf.GetGradient(x_location, y_location, z_location, true); // Usually, you want to enable 'edge gradients' i.e. gradients for cells on the edge of the grid that don't have 6 neighbors
    std::cout << "Location gradient query result - gradient " << PrettyPrint::PrettyPrint(location_gradient_query) << std::endl;
    // Let's display the results to Rviz
    visualization_msgs::Marker sdf_marker = sdf.ExportForDisplay(0.5); // Set the alpha for display
    sdf_marker.ns = "sdf";
    sdf_marker.id = 1;
    visualization_pub.publish(sdf_marker);
    // Let's export the SDF
    sdf_pub.publish(sdf.GetMessageRepresentation());
    std::cout << "...done" << std::endl;
    return 0;
}
