#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/TaggedObjectCollisionMap.h>

#ifndef TAGGED_OBJECT_COLLISION_MAP_HPP
#define TAGGED_OBJECT_COLLISION_MAP_HPP

#define ENABLE_UNORDERED_MAP_SIZE_HINTS

namespace sdf_tools
{
    struct TAGGED_OBJECT_COLLISION_CELL
    {
        float occupancy;
        uint32_t component;
        uint32_t object_id;
        uint32_t convex_segment;

        TAGGED_OBJECT_COLLISION_CELL();

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const uint32_t in_object_id);

        TAGGED_OBJECT_COLLISION_CELL(const float in_occupancy, const uint32_t in_object_id, const uint32_t in_component, const uint32_t in_convex_segment);

        bool SharesConvexSegment(const TAGGED_OBJECT_COLLISION_CELL& other) const;

        std::vector<uint32_t> GetListOfConvexSegments() const;

        bool IsPartOfConvexSegment(const uint32_t segment) const;

        void AddToConvexSegment(const uint32_t segment);

        void RemoveFromConvexSegment(const uint32_t segment);
    };

    std::vector<uint8_t> TaggedObjectCollisionCellToBinary(const TAGGED_OBJECT_COLLISION_CELL& value);

    TAGGED_OBJECT_COLLISION_CELL TaggedObjectCollisionCellFromBinary(const std::vector<uint8_t>& binary);

    class TaggedObjectCollisionMapGrid
    {
    protected:

        static std_msgs::ColorRGBA GenerateComponentColor(const uint32_t component, const float alpha=1.0f);

        bool IsSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        struct bucket_cell
        {
            uint32_t location[3];
            uint32_t closest_point[3];
            double distance_square;
            int32_t update_direction;
        };

        typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

        DistanceField BuildDistanceField(const std::vector<VoxelGrid::GRID_INDEX>& points) const;

        std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods() const;

        int GetDirectionNumber(const int dx, const int dy, const int dz) const;

        double ComputeDistanceSquared(const int32_t x1, const int32_t y1, const int32_t z1, const int32_t x2, const int32_t y2, const int32_t z2) const;

        VoxelGrid::VoxelGrid<TAGGED_OBJECT_COLLISION_CELL> collision_field_;
        uint32_t number_of_components_;
        std::string frame_;
        bool initialized_;
        bool components_valid_;
        bool convex_segments_valid_;

        std::vector<uint8_t> PackBinaryRepresentation(const std::vector<TAGGED_OBJECT_COLLISION_CELL>& raw) const;

        std::vector<TAGGED_OBJECT_COLLISION_CELL> UnpackBinaryRepresentation(const std::vector<uint8_t>& packed) const;

        int64_t MarkConnectedComponent(const int64_t x_index, const int64_t y_index, const int64_t z_index, const uint32_t connected_component);

        std::vector<VoxelGrid::GRID_INDEX> CheckIfConvex(const VoxelGrid::GRID_INDEX& candidate_index, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& explored_indices, const VoxelGrid::VoxelGrid<std::vector<uint32_t>>& region_grid, const uint32_t current_convex_region) const;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TaggedObjectCollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& default_value, const TAGGED_OBJECT_COLLISION_CELL& OOB_value);

        TaggedObjectCollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& default_value, const TAGGED_OBJECT_COLLISION_CELL& OOB_value);

        TaggedObjectCollisionMapGrid(const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& OOB_value);

        TaggedObjectCollisionMapGrid(const Eigen::Isometry3d& origin_transform, const std::string& frame, const double resolution, const double x_size, const double y_size, const double z_size, const TAGGED_OBJECT_COLLISION_CELL& OOB_value);

        TaggedObjectCollisionMapGrid();

        bool IsInitialized() const;

        bool AreComponentsValid() const;

        bool AreConvexSegmentsValid() const;

        std::pair<bool, bool> CheckIfCandidateCorner3d(const Eigen::Vector3d& location) const;

        std::pair<bool, bool> CheckIfCandidateCorner4d(const Eigen::Vector4d& location) const;

        std::pair<bool, bool> CheckIfCandidateCorner(const double x, const double y, const double z) const;

        std::pair<bool, bool> CheckIfCandidateCorner(const VoxelGrid::GRID_INDEX& index) const;

        std::pair<bool, bool> CheckIfCandidateCorner(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable3d(const Eigen::Vector3d& location) const;

        std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable4d(const Eigen::Vector4d& location) const;

        std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const double x, const double y, const double z) const;

        std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const VoxelGrid::GRID_INDEX& index) const;

        std::pair<const TAGGED_OBJECT_COLLISION_CELL&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable3d(const Eigen::Vector3d& location);

        std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable4d(const Eigen::Vector4d& location);

        std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const double x, const double y, const double z);

        std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const VoxelGrid::GRID_INDEX& index);

        std::pair<TAGGED_OBJECT_COLLISION_CELL&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index);

        bool Set(const double x, const double y, const double z, const TAGGED_OBJECT_COLLISION_CELL& value);

        bool Set3d(const Eigen::Vector3d& location, const TAGGED_OBJECT_COLLISION_CELL& value);

        bool Set4d(const Eigen::Vector4d& location, const TAGGED_OBJECT_COLLISION_CELL& value);

        bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const TAGGED_OBJECT_COLLISION_CELL& value);

        bool Set(const VoxelGrid::GRID_INDEX& index, const TAGGED_OBJECT_COLLISION_CELL& value);

        bool Set(const double x, const double y, const double z, TAGGED_OBJECT_COLLISION_CELL&& value);

        bool Set3d(const Eigen::Vector3d& location, TAGGED_OBJECT_COLLISION_CELL&& value);

        bool Set4d(const Eigen::Vector4d& location, TAGGED_OBJECT_COLLISION_CELL&& value);

        bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, TAGGED_OBJECT_COLLISION_CELL&& value);

        bool Set(const VoxelGrid::GRID_INDEX& index, TAGGED_OBJECT_COLLISION_CELL&& value);

        double GetXSize() const;

        double GetYSize() const;

        double GetZSize() const;

        double GetResolution() const;

        TAGGED_OBJECT_COLLISION_CELL GetDefaultValue() const;

        TAGGED_OBJECT_COLLISION_CELL GetOOBValue() const;

        int64_t GetNumXCells() const;

        int64_t GetNumYCells() const;

        int64_t GetNumZCells() const;

        const Eigen::Isometry3d& GetOriginTransform() const;

        const Eigen::Isometry3d& GetInverseOriginTransform() const;

        std::string GetFrame() const;

        std::pair<uint32_t, bool> GetNumConnectedComponents() const;

        std::vector<int64_t> LocationToGridIndex3d(const Eigen::Vector3d& location) const;

        std::vector<int64_t> LocationToGridIndex4d(const Eigen::Vector4d& location) const;

        std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const;

        std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        bool SaveToFile(const std::string& filepath) const;

        bool LoadFromFile(const std::string &filepath);

        sdf_tools::TaggedObjectCollisionMap GetMessageRepresentation() const;

        bool LoadFromMessageRepresentation(const sdf_tools::TaggedObjectCollisionMap& message);

        TaggedObjectCollisionMapGrid Resample(const double new_resolution) const;

        uint32_t UpdateConnectedComponents();

        std::map<uint32_t, std::pair<int32_t, int32_t>> ComputeComponentTopology(const bool ignore_empty_components, const bool recompute_connected_components, const bool verbose);

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>> ExtractComponentSurfaces(const bool ignore_empty_components) const;

        /* Extracts the active indices from a surface map as a vector, which is useful in contexts where a 1-dimensional index into the surface is needed
         */
        std::vector<VoxelGrid::GRID_INDEX> ExtractStaticSurface(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& raw_surface) const;

        std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> ConvertToDynamicSurface(const std::vector<VoxelGrid::GRID_INDEX>& static_surface) const;

        std::unordered_map<VoxelGrid::GRID_INDEX, size_t> BuildSurfaceIndexMap(const std::vector<VoxelGrid::GRID_INDEX>& static_surface) const;

        std::pair<int32_t, int32_t> ComputeHolesInSurface(const uint32_t component, const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const bool verbose) const;

        int32_t ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface_vertex_connectivity) const;

        std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> ExtractSignedDistanceField(const float oob_value, const std::vector<uint32_t>& objects_to_use) const;

        VoxelGrid::VoxelGrid<std::vector<uint32_t>> ComputeConvexRegions(const double max_check_radius) const;

        void GrowConvexRegion(const VoxelGrid::GRID_INDEX& start_index, VoxelGrid::VoxelGrid<std::vector<uint32_t>>& region_grid, const double max_check_radius, const uint32_t current_convex_region) const;

        EigenHelpers::VectorVector3d GenerateRayPrimitiveVectors(const uint32_t number_of_rays, const double cone_angle) const;

        std::pair<std::vector<size_t>, std::vector<size_t>> CastSingleRay(const std::unordered_map<VoxelGrid::GRID_INDEX, size_t>& surface_index_map, const VoxelGrid::GRID_INDEX& current_surface_index, const Eigen::Vector3d& ray_unit_vector) const;

        std::pair<std::vector<size_t>, std::vector<size_t>> PerformRayCasting(const sdf_tools::SignedDistanceField& sdf, const std::unordered_map<VoxelGrid::GRID_INDEX, size_t>& surface_index_map, const VoxelGrid::GRID_INDEX& current_surface_index, const EigenHelpers::VectorVector3d& ray_primitive_vectors) const;

        //std::pair<Eigen::MatrixXd, std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>>> ComputeSparseLineOfSight(const std::vector<VoxelGrid::GRID_INDEX>& static_surface, const uint32_t number_of_rays, const double cone_angle) const
        Eigen::MatrixXd ComputeSparseLineOfSight(const std::vector<VoxelGrid::GRID_INDEX>& static_surface, const uint32_t number_of_rays, const double cone_angle) const;

        std::pair<Eigen::VectorXd, Eigen::MatrixXd> ExtractKLargestEigenvaluesAndEigenvectors(const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType& raw_eigenvalues, const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType& raw_eigenvectors, const uint32_t num_values) const;

        std::vector<uint32_t> PerformKMeansSpectralClustering(const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvalueType& raw_eigenvalues, const Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType& raw_eigenvectors, const uint32_t num_clusters) const;

        double ComputeConvexityMetric(const Eigen::MatrixXd& los_matrix, const std::vector<uint32_t>& cluster_labels) const;

        std::vector<std::vector<size_t>> ClusterSurfaceFromLOSMatrix(const Eigen::MatrixXd& los_matrix, const uint32_t max_num_clusters) const;

        std::vector<std::vector<VoxelGrid::GRID_INDEX>> ComputeWeaklyConvexSurfaceSegments(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const uint32_t max_num_clusters) const;

        std::map<uint32_t, uint32_t> UpdateConvexSegments();

        std::map<uint32_t, sdf_tools::SignedDistanceField> MakeObjectSDFs(const std::vector<uint32_t>& object_ids) const;

        std::map<uint32_t, sdf_tools::SignedDistanceField> MakeObjectSDFs() const;

        visualization_msgs::Marker ExportForDisplay(const float alpha, const std::vector<uint32_t>& objects_to_draw=std::vector<uint32_t>()) const;

        visualization_msgs::Marker ExportForDisplay(const std::map<uint32_t, std_msgs::ColorRGBA>& object_color_map=std::map<uint32_t, std_msgs::ColorRGBA>()) const;

        visualization_msgs::Marker ExportContourOnlyForDisplay(const float alpha, const std::vector<uint32_t>& objects_to_draw=std::vector<uint32_t>()) const;

        visualization_msgs::Marker ExportContourOnlyForDisplay(const std::map<uint32_t, std_msgs::ColorRGBA>& object_color_map=std::map<uint32_t, std_msgs::ColorRGBA>()) const;

        visualization_msgs::Marker ExportForDisplayOccupancyOnly(const std_msgs::ColorRGBA& collision_color, const std_msgs::ColorRGBA& free_color, const std_msgs::ColorRGBA& unknown_color) const;

        visualization_msgs::Marker ExportConnectedComponentsForDisplay(bool color_unknown_components) const;

        visualization_msgs::Marker ExportConvexSegmentForDisplay(const uint32_t object_id, const uint32_t convex_segment) const;

        visualization_msgs::Marker ExportSurfaceForDisplay(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface, const std_msgs::ColorRGBA& surface_color) const;
    };
}

#endif // TAGGED_OBJECT_COLLISION_MAP_HPP
