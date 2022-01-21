#ifndef SDF_HPP
#define SDF_HPP

#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/SDF.h>

namespace sdf_tools
{
    std::vector<uint8_t> FloatToBinary(float value);

    float FloatFromBinary(std::vector<uint8_t>& binary);

    class SignedDistanceField
    {
    protected:

        VoxelGrid::VoxelGrid<float> distance_field_;
        std::string frame_;
        bool initialized_;
        bool locked_;

        std::vector<uint8_t> GetInternalBinaryRepresentation(const std::vector<float> &field_data);

        std::vector<float> UnpackFieldFromBinaryRepresentation(const std::vector<uint8_t>& binary);

        /*
         * You *MUST* provide valid indices to this function, hence why it is protected (there are safe wrappers available - use them!)
         */
        void FollowGradientsToLocalMaximaUnsafe(VoxelGrid::VoxelGrid<Eigen::Vector3d>& watershed_map, const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SignedDistanceField(std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value);

        SignedDistanceField(Eigen::Isometry3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value);

        SignedDistanceField();

        bool IsInitialized() const;

        bool IsLocked() const;

        void Lock();

        void Unlock();

        float Get(const double x, const double y, const double z) const;

        float Get3d(const Eigen::Vector3d& location) const;

        float Get4d(const Eigen::Vector4d& location) const;

        float Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        std::pair<float, bool> GetSafe(const double x, const double y, const double z) const;

        std::pair<float, bool> GetSafe3d(const Eigen::Vector3d& location) const;

        std::pair<float, bool> GetSafe4d(const Eigen::Vector4d& location) const;

        std::pair<float, bool> GetSafe(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        /*
         * Setter functions MUST be used carefully - If you arbitrarily change SDF values, it is not a proper SDF any more!
         *
         * Use of these functions can be prevented by calling SignedDistanceField::Lock() on the SDF, at which point these functions
         * will fail with a warning printed to std_err.
         */
        bool Set(const double x, const double y, const double z, float value);

        bool Set3d(const Eigen::Vector3d& location, float value);

        bool Set4d(const Eigen::Vector4d& location, float value);

        bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const float value);

        bool Set(const VoxelGrid::GRID_INDEX& index, const float value);

        bool CheckInBounds3d(const Eigen::Vector3d& location) const;

        bool CheckInBounds4d(const Eigen::Vector4d& location) const;

        bool CheckInBounds(const double x, const double y, const double z) const;

        bool CheckInBounds(const VoxelGrid::GRID_INDEX& index) const;

        bool CheckInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        double GetXSize() const;

        double GetYSize() const;

        double GetZSize() const;

        double GetResolution() const;

        float GetOOBValue() const;

        int64_t GetNumXCells() const;

        int64_t GetNumYCells() const;

        int64_t GetNumZCells() const;

    protected:

        std::pair<Eigen::Vector3d, double> GetPrimaryComponentsVector(const Eigen::Vector3d& raw_vector) const;

        double ComputeAxisMatch(const double axis_value, const double check_value) const;

        Eigen::Vector3d GetBestMatchSurfaceVector(const Eigen::Vector3d& possible_surfaces_vector, const Eigen::Vector3d& center_to_location_vector) const;

        /**
         * @brief GetPrimaryEntrySurfaceVector Estimates the real distance of the provided point, comparing it with the cell center location and gradient vector
         * @param boundary_direction_vector
         * @param center_to_location_vector
         * @return vector from center of voxel to primary entry surface, and magnitude of that vector
         */
        std::pair<Eigen::Vector3d, double> GetPrimaryEntrySurfaceVector(const Eigen::Vector3d& boundary_direction_vector, const Eigen::Vector3d& center_to_location_vector) const;

        double EstimateDistanceInternal(const double x, const double y, const double z, const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const;

    public:

        std::pair<double, bool> EstimateDistance(const double x, const double y, const double z) const;

        std::pair<double, bool> EstimateDistance3d(const Eigen::Vector3d& location) const;

        std::pair<double, bool> EstimateDistance4d(const Eigen::Vector4d& location) const;

        // Estimate the distance between the given point and the outer boundary of the SDF
        std::pair<double, bool> DistanceToBoundary(const double x, const double y, const double z) const;

        std::pair<double, bool> DistanceToBoundary3d(const Eigen::Vector3d& location) const;

        std::pair<double, bool> DistanceToBoundary4d(const Eigen::Vector4d& location) const;

        std::vector<double> GetGradient(const double x, const double y, const double z, const bool enable_edge_gradients = false) const;

        std::vector<double> GetGradient3d(const Eigen::Vector3d& location, const bool enable_edge_gradients = false) const;

        std::vector<double> GetGradient4d(const Eigen::Vector4d& location, const bool enable_edge_gradients = false) const;

        std::vector<double> GetGradient(const VoxelGrid::GRID_INDEX& index, const bool enable_edge_gradients = false) const;

        std::vector<double> GetGradient(const int64_t x_index, const int64_t y_index, const int64_t z_index, const bool enable_edge_gradients = false) const;

        Eigen::Vector3d ProjectOutOfCollision(const double x, const double y, const double z, const double stepsize_multiplier = 1.0 / 8.0) const;

        Eigen::Vector3d ProjectOutOfCollisionToMinimumDistance(const double x, const double y, const double z, const double minimum_distance, const double stepsize_multiplier = 1.0 / 8.0) const;

        Eigen::Vector3d ProjectOutOfCollision3d(const Eigen::Vector3d& location, const double stepsize_multiplier = 1.0 / 8.0) const;

        Eigen::Vector3d ProjectOutOfCollisionToMinimumDistance3d(const Eigen::Vector3d& location, const double minimum_distance, const double stepsize_multiplier = 1.0 / 8.0) const;

        Eigen::Vector4d ProjectOutOfCollision4d(const Eigen::Vector4d& location, const double stepsize_multiplier = 1.0 / 8.0) const;

        Eigen::Vector4d ProjectOutOfCollisionToMinimumDistance4d(const Eigen::Vector4d& location, const double minimum_distance, const double stepsize_multiplier = 1.0 / 8.0) const;

        Eigen::Vector3d ProjectIntoValidVolume(const double x, const double y, const double z) const;

        Eigen::Vector3d ProjectIntoValidVolumeToMinimumDistance(const double x, const double y, const double z, const double minimum_distance) const;

        Eigen::Vector3d ProjectIntoValidVolume3d(const Eigen::Vector3d& location) const;

        Eigen::Vector3d ProjectIntoValidVolumeToMinimumDistance3d(const Eigen::Vector3d& location, const double minimum_distance) const;

        Eigen::Vector4d ProjectIntoValidVolume4d(const Eigen::Vector4d& location) const;

        Eigen::Vector4d ProjectIntoValidVolumeToMinimumDistance4d(const Eigen::Vector4d& location, const double minimum_distance) const;

        const Eigen::Isometry3d& GetOriginTransform() const;

        const Eigen::Isometry3d& GetInverseOriginTransform() const;

        std::string GetFrame() const;

        std::vector<int64_t> LocationToGridIndex3d(const Eigen::Vector3d& location) const;

        std::vector<int64_t> LocationToGridIndex4d(const Eigen::Vector4d& location) const;

        std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const;

        std::vector<double> GridIndexToLocation(const VoxelGrid::GRID_INDEX& index) const;

        std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const;

        bool SaveToFile(const std::string& filepath);

        bool LoadFromFile(const std::string& filepath);

        sdf_tools::SDF GetMessageRepresentation();

        bool LoadFromMessageRepresentation(const sdf_tools::SDF& message);

        visualization_msgs::Marker ExportForDisplay(const float alpha = 0.01f) const;

        visualization_msgs::Marker ExportForDisplayCollisionOnly(const float alpha = 0.01f) const;

        visualization_msgs::Marker ExportForDebug(const float alpha = 0.5f) const;

        /*
         * The following function can be *VERY EXPENSIVE* to compute, since it performs gradient ascent across the SDF
         */
        VoxelGrid::VoxelGrid<Eigen::Vector3d> ComputeLocalMaximaMap() const;

        bool GradientIsEffectiveFlat(const Eigen::Vector3d& gradient) const;

        VoxelGrid::GRID_INDEX GetNextFromGradient(const VoxelGrid::GRID_INDEX& index, const Eigen::Vector3d& gradient) const;
    };
}

#endif // SDF_HPP
