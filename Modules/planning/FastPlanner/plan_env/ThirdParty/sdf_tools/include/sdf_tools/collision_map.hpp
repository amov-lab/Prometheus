#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/CollisionMap.h>

#ifndef COLLISION_MAP_HPP
#define COLLISION_MAP_HPP

#define ENABLE_UNORDERED_MAP_SIZE_HINTS

namespace sdf_tools
{
    struct COLLISION_CELL
    {
        float occupancy;
        uint32_t component;

        COLLISION_CELL(const float in_occupancy = 0.0, const uint32_t in_component = 0);
    };

    std::vector<uint8_t> CollisionCellToBinary(const COLLISION_CELL& value);

    COLLISION_CELL CollisionCellFromBinary(const std::vector<uint8_t>& binary);

    class CollisionMapGrid
    {
    protected:

        static std_msgs::ColorRGBA GenerateComponentColor(const uint32_t component, const float alpha=1.0f);

        bool IsSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        typedef struct
        {
            uint32_t location[3];
            uint32_t closest_point[3];
            double distance_square;
            int32_t update_direction;
        } bucket_cell;

        typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

        DistanceField BuildDistanceField(const std::vector<VoxelGrid::GRID_INDEX>& points) const;

        std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods() const;

        int GetDirectionNumber(const int dx, const int dy, const int dz) const;

        double ComputeDistanceSquared(const int32_t x1, const int32_t y1, const int32_t z1, const int32_t x2, const int32_t y2, const int32_t z2) const;

        VoxelGrid::VoxelGrid<COLLISION_CELL> collision_field_;
        uint32_t number_of_components_;
        std::string frame_;
        bool initialized_;
        bool components_valid_;

        std::vector<uint8_t> PackBinaryRepresentation(std::vector<COLLISION_CELL>& raw);

        std::vector<COLLISION_CELL> UnpackBinaryRepresentation(std::vector<uint8_t>& packed);

        int64_t MarkConnectedComponent(int64_t x_index, int64_t y_index, int64_t z_index, uint32_t connected_component);

    public:

        CollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const COLLISION_CELL& default_value, const COLLISION_CELL& OOB_value);

        CollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, double y_size, const double z_size, const COLLISION_CELL& default_value, const COLLISION_CELL& OOB_value);

        CollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const COLLISION_CELL& OOB_default_value);

        CollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, double y_size, const double z_size, const COLLISION_CELL& OOB_default_value);

        CollisionMapGrid();

        bool IsInitialized() const;

        bool AreComponentsValid() const;

        std::pair<COLLISION_CELL, bool> Get3d(const Eigen::Vector3d& location) const;

        std::pair<COLLISION_CELL, bool> Get4d(const Eigen::Vector4d& location) const;

        std::pair<COLLISION_CELL, bool> Get(const double x, const double y, const double z) const;

        std::pair<COLLISION_CELL, bool> Get(const VoxelGrid::GRID_INDEX& index) const;

        std::pair<COLLISION_CELL, bool> Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        bool Set(const double x, const double y, const double z, COLLISION_CELL value);

        bool Set3d(const Eigen::Vector3d& location, COLLISION_CELL value);

        bool Set4d(const Eigen::Vector4d& location, COLLISION_CELL value);

        bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, COLLISION_CELL value);

        bool Set(const VoxelGrid::GRID_INDEX& index, COLLISION_CELL value);

        double GetXSize() const;

        double GetYSize() const;

        double GetZSize() const;

        double GetResolution() const;

        COLLISION_CELL GetDefaultValue() const;

        COLLISION_CELL GetOOBValue() const;

        int64_t GetNumXCells() const;

        int64_t GetNumYCells() const;

        int64_t GetNumZCells() const;

        const Eigen::Isometry3d& GetOriginTransform() const;

        const Eigen::Isometry3d& GetInverseOriginTransform() const;

        std::string GetFrame() const;

        std::pair<uint32_t, bool> GetNumConnectedComponents() const;

        std::vector<int64_t> LocationToGridIndex3d(const Eigen::Vector3d& location) const;

        std::vector<int64_t> LocationToGridIndex4d(const Eigen::Vector4d& location) const;

        std::vector<int64_t> LocationToGridIndex(double x, double y, double z) const;

        std::vector<double> GridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index) const;

        bool SaveToFile(const std::string& filepath);

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::CollisionMap GetMessageRepresentation();

        bool LoadFromMessageRepresentation(sdf_tools::CollisionMap& message);

        uint32_t UpdateConnectedComponents();

        std::map<uint32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(bool ignore_empty_components, bool recompute_connected_components, bool verbose);

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> ExtractComponentSurfaces(const bool ignore_empty_components) const;

        std::pair<int32_t, int32_t> ComputeHolesInSurface(const uint32_t component, const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const bool verbose) const;

        int32_t ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface_vertex_connectivity) const;

        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const float oob_value) const;

        visualization_msgs::Marker ExportForDisplay(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;
    };
}

#endif // COLLISION_MAP_HPP
