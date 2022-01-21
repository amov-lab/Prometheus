#include "arc_utilities/voxel_grid.hpp"
#include "arc_utilities/pretty_print.hpp"
#include "sdf_tools/collision_map.hpp"
#include "arc_utilities/dynamic_spatial_hashed_voxel_grid.hpp"
#include "sdf_tools/dynamic_spatial_hashed_collision_map.hpp"
#include "sdf_tools/sdf.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <chrono>
#include <random>

void test_voxel_grid_indices()
{
    VoxelGrid::VoxelGrid<int> test_grid(1.0, 20.0, 20.0, 20.0, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                test_grid.SetValue(x_index, y_index, z_index, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                int ref_val = test_grid.GetImmutable(x_index, y_index, z_index).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Check pass" << std::endl;
                }
                else
                {
                    std::cout << "Check fail" << std::endl;
                    pass = false;
                }
                check_index++;
            }
        }
    }
    if (pass)
    {
        std::cout << "VG-I - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** VG-I - Checks failed ***" << std::endl;
    }
}

void test_voxel_grid_locations()
{
    VoxelGrid::VoxelGrid<int> test_grid(1.0, 20.0, 20.0, 20.0, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                test_grid.SetValue(x_pos, y_pos, z_pos, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                int ref_val = test_grid.GetImmutable(x_pos, y_pos, z_pos).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Value check pass" << std::endl;
                }
                else
                {
                    std::cout << "Value check fail" << std::endl;
                    pass = false;
                }
                check_index++;
                std::vector<double> query_point = {x_pos, y_pos, z_pos};
                //std::cout << "Query point - " << PrettyPrint::PrettyPrint(query_point) << std::endl;
                std::vector<int64_t> query_index = test_grid.LocationToGridIndex(x_pos, y_pos, z_pos);
                //std::cout << "Query index - " << PrettyPrint::PrettyPrint(query_index) << std::endl;
                std::vector<double> query_location = test_grid.GridIndexToLocation(query_index[0], query_index[1], query_index[2]);
                //std::cout << "Query location - " << PrettyPrint::PrettyPrint(query_location) << std::endl;
                std::vector<int64_t> found_query_index = test_grid.LocationToGridIndex(query_location[0], query_location[1], query_location[2]);
                //std::cout << "Found query index - " << PrettyPrint::PrettyPrint(found_query_index) << std::endl;
                if (query_point[0] == query_location[0] && query_point[1] == query_location[1] && query_point[2] == query_location[2])
                {
                    //std::cout << "Position check pass" << std::endl;
                }
                else
                {
                    std::cout << "Position check fail" << std::endl;
                    pass = false;
                }
                if (query_index[0] == found_query_index[0] && query_index[1] == found_query_index[1] && query_index[2] == found_query_index[2])
                {
                    //std::cout << "Position index check pass" << std::endl;
                }
                else
                {
                    std::cout << "Position index check fail" << std::endl;
                    pass = false;
                }
            }
        }
    }
    if (pass)
    {
        std::cout << "VG-L - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** VG-L - Checks failed ***" << std::endl;
    }
}

void test_voxel_grid_serialization()
{
    VoxelGrid::VoxelGrid<int> test_grid(1.0, 20.0, 20.0, 20.0, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                test_grid.SetValue(x_index, y_index, z_index, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    std::vector<uint8_t> buffer;
    VoxelGrid::VoxelGrid<int>::Serialize(test_grid, buffer, arc_utilities::SerializeFixedSizePOD<int>);
    const VoxelGrid::VoxelGrid<int> read_grid = VoxelGrid::VoxelGrid<int>::Deserialize(buffer, 0, arc_utilities::DeserializeFixedSizePOD<int>).first;
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (int64_t x_index = 0; x_index < read_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < read_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < read_grid.GetNumZCells(); z_index++)
            {
                int ref_val = read_grid.GetImmutable(x_index, y_index, z_index).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Check pass" << std::endl;
                }
                else
                {
                    std::cout << "Check fail" << std::endl;
                    pass = false;
                }
                check_index++;
            }
        }
    }
    if (pass)
    {
        std::cout << "VG-I de/serialize - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** VG-I de/serialize - Checks failed ***" << std::endl;
    }
}

void test_dsh_voxel_grid_locations()
{
    VoxelGrid::DynamicSpatialHashedVoxelGrid<int> test_grid(1.0, 4, 4, 4, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                test_grid.SetCellValue(x_pos, y_pos, z_pos, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                int ref_val = test_grid.GetImmutable(x_pos, y_pos, z_pos).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Value check pass" << std::endl;
                }
                else
                {
                    std::cout << "Value check fail" << std::endl;
                    pass = false;
                }
                check_index++;
            }
        }
    }
    if (pass)
    {
        std::cout << "DSHVG - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** DSHVG - Checks failed ***" << std::endl;
    }
}


void test_float_binary_conversion(float test_val)
{
    std::cout << "Initial value " << test_val << std::endl;
    std::vector<uint8_t> binary_value = sdf_tools::FloatToBinary(test_val);
    float final_val = sdf_tools::FloatFromBinary(binary_value);
    std::cout << "Final value " << final_val << std::endl;
}

Eigen::Vector3d get_random_location(std::default_random_engine& generator, const double min_x, const double min_y, const double min_z, const double max_x, const double max_y, const double max_z)
{
    std::uniform_real_distribution<double> x_distribution(min_x, max_x);
    std::uniform_real_distribution<double> y_distribution(min_y, max_y);
    std::uniform_real_distribution<double> z_distribution(min_z, max_z);
    double rand_x = x_distribution(generator);
    double rand_y = y_distribution(generator);
    double rand_z = z_distribution(generator);
    return Eigen::Vector3d(rand_x, rand_y, rand_z);
}

bool get_random_bool(std::default_random_engine& generator)
{
    std::uniform_int_distribution<int> distribution(0,1);
    int rand_int = distribution(generator);
    if (rand_int)
    {
        return true;
    }
    else
    {
        return false;
    }
}

visualization_msgs::MarkerArray test_dsh_collision_map(std::default_random_engine& generator)
{
    sdf_tools::COLLISION_CELL default_cell(0.0);
    sdf_tools::COLLISION_CELL filled_cell(1.0);
    sdf_tools::DynamicSpatialHashedCollisionMapGrid test_col_map("test_voxel_grid", 1.0, 5, 5, 5, default_cell);
    // Add a bunch of random data
    for (int idx = 0; idx < 500; idx++)
    {
        // Get a random location in +-10m
        Eigen::Vector3d random_location = get_random_location(generator, -20.0, -20.0, -20.0, 20.0, 20.0, 20.0);
        // Get a random bool to choose between cell/chunk
        bool use_cell = get_random_bool(generator);
        // Update the col map
        if (use_cell)
        {
            test_col_map.SetCell(random_location, filled_cell);
        }
        else
        {
            test_col_map.SetChunk(random_location, filled_cell);
        }
    }
    // Get the Rviz markers
    std_msgs::ColorRGBA filled_color;
    filled_color.a = 1.0;
    filled_color.b = 0.0;
    filled_color.g = 0.0;
    filled_color.r = 1.0;
    std_msgs::ColorRGBA free_color;
    free_color.a = 0.1;
    free_color.b = 0.0;
    free_color.g = 1.0;
    free_color.r = 0.0;
    std_msgs::ColorRGBA unknown_color;
    unknown_color.a = 0.5;
    unknown_color.b = 1.0;
    unknown_color.g = 0.0;
    unknown_color.r = 0.0;
    std::vector<visualization_msgs::Marker> display_markers = test_col_map.ExportForDisplay(filled_color, free_color, unknown_color);
    visualization_msgs::MarkerArray display_rep;
    display_rep.markers = display_markers;
    return display_rep;
}


int main(int argc, char** argv)
{
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    ros::init(argc, argv, "test_voxel_grid");
    ros::NodeHandle nh;
    ros::Publisher display_pub = nh.advertise<visualization_msgs::MarkerArray>("display_test_voxel_grid", 1, true);
    test_voxel_grid_indices();
    test_voxel_grid_locations();
    test_voxel_grid_serialization();
    test_dsh_voxel_grid_locations();
    test_float_binary_conversion(5280.0);
    visualization_msgs::MarkerArray display_rep = test_dsh_collision_map(generator);
    display_pub.publish(display_rep);
    ros::spin();
    return 0;
}
