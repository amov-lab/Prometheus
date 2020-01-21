#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <zlib.h>
#include <ros/ros.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/SDF.h>

namespace sdf_tools
{
    std::vector<uint8_t> FloatToBinary(float value)
    {
        uint32_t binary_value = 0;
        memcpy(&binary_value, &value, sizeof(uint32_t));
        std::vector<uint8_t> binary(4);
        // Copy byte 1, least-significant byte
        binary[3] = binary_value & 0x000000ff;
        // Copy byte 2
        binary_value = binary_value >> 8;
        binary[2] = binary_value & 0x000000ff;
        // Copy byte 3
        binary_value = binary_value >> 8;
        binary[1] = binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        binary_value = binary_value >> 8;
        binary[0] = binary_value & 0x000000ff;
        return binary;
    }

    float FloatFromBinary(std::vector<uint8_t>& binary)
    {
        if (binary.size() != 4)
        {
            std::cerr << "Binary value is not 4 bytes" << std::endl;
            return NAN;
        }
        else
        {
            uint32_t binary_value = 0;
            // Copy in byte 4, most-significant byte
            binary_value = binary_value | binary[0];
            binary_value = binary_value << 8;
            // Copy in byte 3
            binary_value = binary_value | binary[1];
            binary_value = binary_value << 8;
            // Copy in byte 2
            binary_value = binary_value | binary[2];
            binary_value = binary_value << 8;
            // Copy in byte 1, least-significant byte
            binary_value = binary_value | binary[3];
            // Convert binary to float and store
            float field_value = 0.0;
            memcpy(&field_value, &binary_value, sizeof(float));
            return field_value;
        }
    }



    std::vector<uint8_t> SignedDistanceField::GetInternalBinaryRepresentation(const std::vector<float>& field_data)
    {
        std::vector<uint8_t> raw_binary_data(field_data.size() * 4);
        for (size_t field_index = 0, binary_index = 0; field_index < field_data.size(); field_index++, binary_index+=4)
        {
            // Convert the float at the current index into 4 bytes and store them
            float field_value = field_data[field_index];
            std::vector<uint8_t> binary_value = FloatToBinary(field_value);
            raw_binary_data[binary_index] = binary_value[0];
            raw_binary_data[binary_index + 1] = binary_value[1];
            raw_binary_data[binary_index + 2] = binary_value[2];
            raw_binary_data[binary_index + 3] = binary_value[3];
        }
        return raw_binary_data;
    }

    std::vector<float> SignedDistanceField::UnpackFieldFromBinaryRepresentation(const std::vector<uint8_t>& binary)
    {
        if ((binary.size() % 4) != 0)
        {
            std::cerr << "Invalid binary representation - length is not a multiple of 4" << std::endl;
            return std::vector<float>();
        }
        uint64_t data_size = binary.size() / 4;
        std::vector<float> field_data(data_size);
        for (size_t field_index = 0, binary_index = 0; field_index < field_data.size(); field_index++, binary_index+=4)
        {
            std::vector<uint8_t> binary_block{binary[binary_index], binary[binary_index + 1], binary[binary_index + 2], binary[binary_index + 3]};
            field_data[field_index] = FloatFromBinary(binary_block);
        }
        return field_data;
    }

    void SignedDistanceField::FollowGradientsToLocalMaximaUnsafe(VoxelGrid::VoxelGrid<Eigen::Vector3d>& watershed_map, const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        // First, check if we've already found the local maxima for the current cell
        const Eigen::Vector3d& stored = watershed_map.GetImmutable(x_index, y_index, z_index).first;
        if (stored.x() != -INFINITY && stored.y() != -INFINITY && stored.z() != -INFINITY)
        {
            // We've already found it for this cell, so we can skip it
            return;
        }
        // Second, check if it's inside an obstacle
        float stored_distance = Get(x_index, y_index, z_index);
        if (stored_distance <= 0.0)
        {
            // It's inside an object, so we can skip it
            return;
        }
        else
        {
            // Find the local maxima
            std::vector<double> raw_gradient = GetGradient(x_index, y_index, z_index, true);
            Eigen::Vector3d current_gradient(raw_gradient[0], raw_gradient[1], raw_gradient[2]);
            if (GradientIsEffectiveFlat(current_gradient))
            {
                std::vector<double> location = GridIndexToLocation(x_index, y_index, z_index);
                Eigen::Vector3d local_maxima(location[0], location[1], location[2]);
                watershed_map.SetValue(x_index, y_index, z_index, local_maxima);
            }
            else
            {
                // Follow the gradient, one cell at a time, until we reach a local maxima
                std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> path;
                VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                path[current_index] = 1;
                Eigen::Vector3d local_maxima(-INFINITY, -INFINITY, -INFINITY);
                while (true)
                {
                    if (path.size() == 10000)
                    {
                        std::cerr << "Warning, gradient path is long (i.e >= 10000 steps)" << std::endl;
                    }
                    current_index = GetNextFromGradient(current_index, current_gradient);
                    if (path[current_index] != 0)
                    {
                        //std::cerr << "LMAX found by cycle detect" << std::endl;
                        // If we've already been here, then we are done
                        std::vector<double> location = GridIndexToLocation(current_index);
                        local_maxima = Eigen::Vector3d(location[0], location[1], location[2]);
                        break;
                    }
                    // Check if we've been pushed past the edge
                    if (current_index.x < 0 || current_index.y < 0 || current_index.z < 0 || current_index.x >= watershed_map.GetNumXCells() || current_index.y >= watershed_map.GetNumYCells() || current_index.z >= watershed_map.GetNumZCells())
                    {
                        // We have the "off the grid" local maxima
                        local_maxima = Eigen::Vector3d(INFINITY, INFINITY, INFINITY);
                        break;
                    }
                    path[current_index] = 1;
                    // Check if the new index has already been checked
                    const Eigen::Vector3d& new_stored = watershed_map.GetImmutable(current_index).first;
                    if (new_stored.x() != -INFINITY && new_stored.y() != -INFINITY && new_stored.z() != -INFINITY)
                    {
                        // We have the local maxima
                        local_maxima = new_stored;
                        break;
                    }
                    else
                    {
                        raw_gradient = GetGradient(current_index, true);
                        current_gradient = Eigen::Vector3d(raw_gradient[0], raw_gradient[1], raw_gradient[2]);
                        if (GradientIsEffectiveFlat(current_gradient))
                        {
                            //std::cerr << "LMAX found by flat detect" << std::endl;
                            // We have the local maxima
                            std::vector<double> location = GridIndexToLocation(current_index);
                            local_maxima = Eigen::Vector3d(location[0], location[1], location[2]);
                            break;
                        }
                    }
                }
                // Now, go back and mark the entire explored path with the local maxima
                std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>::const_iterator path_itr;
                for (path_itr = path.begin(); path_itr != path.end(); ++path_itr)
                {
                    const VoxelGrid::GRID_INDEX& index = path_itr->first;
                    watershed_map.SetValue(index, local_maxima);
                }
            }
        }
    }

    SignedDistanceField::SignedDistanceField(std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value)
        : initialized_(true), locked_(false)
    {
        frame_ = frame;
        VoxelGrid::VoxelGrid<float> new_field(resolution, x_size, y_size, z_size, OOB_value);
        distance_field_ = new_field;
    }

    SignedDistanceField::SignedDistanceField(Eigen::Isometry3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, float OOB_value)
        : initialized_(true), locked_(false)
    {
        frame_ = frame;
        VoxelGrid::VoxelGrid<float> new_field(origin_transform, resolution, x_size, y_size, z_size, OOB_value);
        distance_field_ = new_field;
    }

    SignedDistanceField::SignedDistanceField()
        : initialized_(false), locked_(false) {}

    bool SignedDistanceField::IsInitialized() const
    {
        return initialized_;
    }

    bool SignedDistanceField::IsLocked() const
    {
        return locked_;
    }

    void SignedDistanceField::Lock()
    {
        locked_ = true;
    }

    void SignedDistanceField::Unlock()
    {
        locked_ = false;
    }

    float SignedDistanceField::Get(const double x, const double y, const double z) const
    {
        return distance_field_.GetImmutable(x, y, z).first;
    }

    float SignedDistanceField::Get3d(const Eigen::Vector3d& location) const
    {
        return distance_field_.GetImmutable3d(location).first;
    }

    float SignedDistanceField::Get4d(const Eigen::Vector4d& location) const
    {
        return distance_field_.GetImmutable4d(location).first;
    }

    float SignedDistanceField::Get(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        return distance_field_.GetImmutable(x_index, y_index, z_index).first;
    }

    std::pair<float, bool> SignedDistanceField::GetSafe(const double x, const double y, const double z) const
    {
        return distance_field_.GetImmutable(x, y, z);
    }

    std::pair<float, bool> SignedDistanceField::GetSafe3d(const Eigen::Vector3d& location) const
    {
        return distance_field_.GetImmutable3d(location);
    }

    std::pair<float, bool> SignedDistanceField::GetSafe4d(const Eigen::Vector4d& location) const
    {
        return distance_field_.GetImmutable4d(location);
    }

    std::pair<float, bool> SignedDistanceField::GetSafe(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        return distance_field_.GetImmutable(x_index, y_index, z_index);
    }

    /*
     * Setter functions MUST be used carefully - If you arbitrarily change SDF values, it is not a proper SDF any more!
     *
     * Use of these functions can be prevented by calling SignedDistanceField::Lock() on the SDF, at which point these functions
     * will fail with a warning printed to std_err.
     */
    bool SignedDistanceField::Set(const double x, const double y, const double z, float value)
    {
        if (!locked_)
        {
            return distance_field_.SetValue(x, y, z, value);
        }
        else
        {
            std::cerr << "Attempt to set value in locked SDF" << std::endl;
            return false;
        }
    }

    bool SignedDistanceField::Set3d(const Eigen::Vector3d& location, float value)
    {
        if (!locked_)
        {
            return distance_field_.SetValue3d(location, value);
        }
        else
        {
            std::cerr << "Attempt to set value in locked SDF" << std::endl;
            return false;
        }
    }

    bool SignedDistanceField::Set4d(const Eigen::Vector4d& location, float value)
    {
        if (!locked_)
        {
            return distance_field_.SetValue4d(location, value);
        }
        else
        {
            std::cerr << "Attempt to set value in locked SDF" << std::endl;
            return false;
        }
    }

    bool SignedDistanceField::Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, const float value)
    {
        if (!locked_)
        {
            return distance_field_.SetValue(x_index, y_index, z_index, value);
        }
        else
        {
            std::cerr << "Attempt to set value in locked SDF" << std::endl;
            return false;
        }
    }

    bool SignedDistanceField::Set(const VoxelGrid::GRID_INDEX& index, const float value)
    {
        if (!locked_)
        {
            return distance_field_.SetValue(index, value);
        }
        else
        {
            std::cerr << "Attempt to set value in locked SDF" << std::endl;
            return false;
        }
    }

    bool SignedDistanceField::CheckInBounds3d(const Eigen::Vector3d& location) const
    {
        return distance_field_.GetImmutable3d(location).second;
    }

    bool SignedDistanceField::CheckInBounds4d(const Eigen::Vector4d& location) const
    {
        return distance_field_.GetImmutable4d(location).second;
    }

    bool SignedDistanceField::CheckInBounds(const double x, const double y, const double z) const
    {
        return distance_field_.GetImmutable(x, y, z).second;
    }

    bool SignedDistanceField::CheckInBounds(const VoxelGrid::GRID_INDEX& index) const
    {
        return distance_field_.GetImmutable(index.x, index.y, index.z).second;
    }

    bool SignedDistanceField::CheckInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        return distance_field_.GetImmutable(x_index, y_index, z_index).second;
    }

    double SignedDistanceField::GetXSize() const
    {
        return distance_field_.GetXSize();
    }

    double SignedDistanceField::GetYSize() const
    {
        return distance_field_.GetYSize();
    }

    double SignedDistanceField::GetZSize() const
    {
        return distance_field_.GetZSize();
    }

    double SignedDistanceField::GetResolution() const
    {
        return distance_field_.GetCellSizes()[0];
    }

    float SignedDistanceField::GetOOBValue() const
    {
        return distance_field_.GetDefaultValue();
    }

    int64_t SignedDistanceField::GetNumXCells() const
    {
        return distance_field_.GetNumXCells();
    }

    int64_t SignedDistanceField::GetNumYCells() const
    {
        return distance_field_.GetNumYCells();
    }

    int64_t SignedDistanceField::GetNumZCells() const
    {
        return distance_field_.GetNumZCells();
    }

    std::pair<Eigen::Vector3d, double> SignedDistanceField::GetPrimaryComponentsVector(const Eigen::Vector3d& raw_vector) const
    {
        if (std::abs(raw_vector.x()) > std::abs(raw_vector.y()) && std::abs(raw_vector.x()) > std::abs(raw_vector.z()))
        {
            if (raw_vector.x() >= 0.0)
            {
                return std::make_pair(Eigen::Vector3d(GetResolution() * 0.5, 0.0, 0.0), GetResolution() * 0.5);
            }
            else
            {
                return std::make_pair(Eigen::Vector3d(GetResolution() * -0.5, 0.0, 0.0), GetResolution() * 0.5);
            }
        }
        else if (std::abs(raw_vector.y()) > std::abs(raw_vector.x()) && std::abs(raw_vector.y()) > std::abs(raw_vector.z()))
        {
            if (raw_vector.y() >= 0.0)
            {
                return std::make_pair(Eigen::Vector3d(0.0, GetResolution() * 0.5, 0.0), GetResolution() * 0.5);
            }
            else
            {
                return std::make_pair(Eigen::Vector3d(0.0, GetResolution() * -0.5, 0.0), GetResolution() * 0.5);
            }
        }
        else if (std::abs(raw_vector.z()) > std::abs(raw_vector.x()) && std::abs(raw_vector.z()) > std::abs(raw_vector.y()))
        {
            if (raw_vector.z() >= 0.0)
            {
                return std::make_pair(Eigen::Vector3d(0.0, 0.0, GetResolution() * 0.5), GetResolution() * 0.5);
            }
            else
            {
                return std::make_pair(Eigen::Vector3d(0.0, 0.0, GetResolution() * -0.5), GetResolution() * 0.5);
            }
        }
        else if (std::abs(raw_vector.x()) == std::abs(raw_vector.y()))
        {
            const Eigen::Vector3d temp_vector(raw_vector.x(), raw_vector.y(), 0.0);
            return std::make_pair((temp_vector / (temp_vector.norm())) * std::sqrt((GetResolution() * GetResolution() * 0.25) * 2.0), std::sqrt((GetResolution() * GetResolution() * 0.25) * 2.0));
        }
        else if (std::abs(raw_vector.y()) == std::abs(raw_vector.z()))
        {
            const Eigen::Vector3d temp_vector(0.0, raw_vector.y(), raw_vector.x());
            return std::make_pair((temp_vector / (temp_vector.norm())) * std::sqrt((GetResolution() * GetResolution() * 0.25) * 2.0), std::sqrt((GetResolution() * GetResolution() * 0.25) * 2.0));
        }
        else if (std::abs(raw_vector.x()) == std::abs(raw_vector.z()))
        {
            const Eigen::Vector3d temp_vector(raw_vector.x(), 0.0, raw_vector.z());
            return std::make_pair((temp_vector / (temp_vector.norm())) * std::sqrt((GetResolution() * GetResolution() * 0.25) * 2.0), std::sqrt((GetResolution() * GetResolution() * 0.25) * 2.0));
        }
        else
        {
            return std::make_pair((raw_vector / (raw_vector.norm())) * std::sqrt((GetResolution() * GetResolution() * 0.25) * 3.0), std::sqrt((GetResolution() * GetResolution() * 0.25) * 3.0));
        }
    }

    double SignedDistanceField::ComputeAxisMatch(const double axis_value, const double check_value) const
    {
        if ((axis_value >= 0.0) == (check_value >= 0.0))
        {
            return std::abs(check_value - axis_value);
        }
        else
        {
            return -std::abs(check_value - axis_value);
        }
    }

    Eigen::Vector3d SignedDistanceField::GetBestMatchSurfaceVector(const Eigen::Vector3d& possible_surfaces_vector, const Eigen::Vector3d& center_to_location_vector) const
    {
        const Eigen::Vector3d location_rejected_on_possible = EigenHelpers::VectorRejection(possible_surfaces_vector, center_to_location_vector);
        // Find the axis with the best-match components
        const double x_axis_match = ComputeAxisMatch(possible_surfaces_vector.x(), location_rejected_on_possible.x());
        const double y_axis_match = ComputeAxisMatch(possible_surfaces_vector.y(), location_rejected_on_possible.y());
        const double z_axis_match = ComputeAxisMatch(possible_surfaces_vector.z(), location_rejected_on_possible.z());
        // Cases where one is better
        if ((x_axis_match > y_axis_match) && (x_axis_match > z_axis_match))
        {
            return Eigen::Vector3d(possible_surfaces_vector.x(), 0.0, 0.0);
        }
        else if ((y_axis_match > x_axis_match) && (y_axis_match > z_axis_match))
        {
            return Eigen::Vector3d(0.0, possible_surfaces_vector.y(), 0.0);
        }
        else if ((z_axis_match > x_axis_match) && (z_axis_match > y_axis_match))
        {
            return Eigen::Vector3d(0.0, 0.0, possible_surfaces_vector.z());
        }
        // Cases where two are equally good
        else if ((x_axis_match < y_axis_match) && (x_axis_match < z_axis_match))
        {
            return Eigen::Vector3d(0.0, possible_surfaces_vector.y(), possible_surfaces_vector.z());
        }
        else if ((y_axis_match < x_axis_match) && (y_axis_match < z_axis_match))
        {
            return Eigen::Vector3d(possible_surfaces_vector.x(), 0.0, possible_surfaces_vector.z());
        }
        else if ((z_axis_match < x_axis_match) && (z_axis_match < y_axis_match))
        {
            return Eigen::Vector3d(possible_surfaces_vector.x(), possible_surfaces_vector.y(), 0.0);
        }
        // When all are equally good
        else
        {
            std::cerr << "Possible surfaces vector " << possible_surfaces_vector << " simple match failed: " << x_axis_match << ", " << y_axis_match << ", " << z_axis_match << " (x, y, z)" << std::endl;
            return possible_surfaces_vector;
        }
    }

    /**
     * @brief GetPrimaryEntrySurfaceVector Estimates the real distance of the provided point, comparing it with the cell center location and gradient vector
     * @param boundary_direction_vector
     * @param center_to_location_vector
     * @return vector from center of voxel to primary entry surface, and magnitude of that vector
     */
    std::pair<Eigen::Vector3d, double> SignedDistanceField::GetPrimaryEntrySurfaceVector(const Eigen::Vector3d& boundary_direction_vector, const Eigen::Vector3d& center_to_location_vector) const
    {
        if (boundary_direction_vector.squaredNorm() > std::numeric_limits<double>::epsilon())
        {
            const std::pair<Eigen::Vector3d, double> primary_components_vector_query = GetPrimaryComponentsVector(boundary_direction_vector);
            // If the cell is on a surface
            if (primary_components_vector_query.second == (GetResolution() * 0.5))
            {
                return primary_components_vector_query;
            }
            // If the cell is on an edge or surface
            else
            {
                // Pick the best-match of the two/three exposed surfaces
                return std::make_pair(GetBestMatchSurfaceVector(primary_components_vector_query.first, center_to_location_vector), GetResolution() * 0.5);
            }
        }
        else
        {
            return GetPrimaryComponentsVector(center_to_location_vector);
        }
    }

    double SignedDistanceField::EstimateDistanceInternal(const double x, const double y, const double z, const int64_t x_idx, const int64_t y_idx, const int64_t z_idx) const
    {
        const std::vector<double> cell_center = GridIndexToLocation(x_idx, y_idx, z_idx);
        const Eigen::Vector3d cell_center_to_location_vector(x - cell_center[0], y - cell_center[1], z - cell_center[2]);
        const double nominal_sdf_distance = (double)distance_field_.GetImmutable(x_idx, y_idx, z_idx).first;

        // Determine vector from "entry surface" to center of voxel
        // TODO: Needs special handling if there's no gradient to work with
        const std::vector<double> raw_gradient = GetGradient(x_idx, y_idx, z_idx, true);
        const Eigen::Vector3d gradient = EigenHelpers::StdVectorDoubleToEigenVector3d(raw_gradient);
        const Eigen::Vector3d direction_to_boundary = (nominal_sdf_distance >= 0.0) ? -gradient : gradient;
        const std::pair<Eigen::Vector3d, double> entry_surface_information = GetPrimaryEntrySurfaceVector(direction_to_boundary, cell_center_to_location_vector);
        const Eigen::Vector3d& entry_surface_vector = entry_surface_information.first;
        const double minimum_distance_magnitude = entry_surface_information.second;

        // Adjust for calculating distance to boundary of voxels instead of center of voxels
        const double center_adjusted_nominal_distance = (nominal_sdf_distance >= 0.0) ? nominal_sdf_distance - (GetResolution() * 0.5) : nominal_sdf_distance + (GetResolution() * 0.5);
        const double minimum_adjusted_distance = arc_helpers::SpreadValue(center_adjusted_nominal_distance, -minimum_distance_magnitude, 0.0, minimum_distance_magnitude);

        // Account for target location being not at the exact center of the voxel
        const double raw_distance_adjustment = EigenHelpers::VectorProjection(entry_surface_vector, cell_center_to_location_vector).norm();
        const double real_distance_adjustment = (minimum_adjusted_distance >= 0.0) ? -raw_distance_adjustment: raw_distance_adjustment;
        const double final_adjusted_distance = minimum_adjusted_distance + real_distance_adjustment;

        // Perform minimum distance thresholding and error checking
        // TODO: do we need to address this magic number somehow?
        if (std::abs(final_adjusted_distance) < GetResolution() * 0.001)
        {
            return 0.0;
        }
        if ((minimum_adjusted_distance >= 0.0) == (final_adjusted_distance >= 0.0))
        {
            return final_adjusted_distance;
        }
        else
        {
            std::cerr << "Center adjusted nominal distance " << minimum_adjusted_distance << " final adjusted_distance " << final_adjusted_distance << std::endl;
            assert(false && "Mismatched minimum and final adjusted distance signs");
        }
    }

    std::pair<double, bool> SignedDistanceField::EstimateDistance(const double x, const double y, const double z) const
    {
        return EstimateDistance4d(Eigen::Vector4d(x, y, z, 1.0));
    }

    std::pair<double, bool> SignedDistanceField::EstimateDistance3d(const Eigen::Vector3d& location) const
    {
        const std::vector<int64_t> indices = LocationToGridIndex3d(location);
        if (indices.size() == 3)
        {
            return std::make_pair(EstimateDistanceInternal(location.x(), location.y(), location.z(), indices[0], indices[1], indices[2]), true);
        }
        else
        {
            return std::make_pair((double)distance_field_.GetOOBValue(), false);
        }
    }

    std::pair<double, bool> SignedDistanceField::EstimateDistance4d(const Eigen::Vector4d& location) const
    {
        const std::vector<int64_t> indices = LocationToGridIndex4d(location);
        if (indices.size() == 3)
        {
            return std::make_pair(EstimateDistanceInternal(location(0), location(1), location(2), indices[0], indices[1], indices[2]), true);
        }
        else
        {
            return std::make_pair((double)distance_field_.GetOOBValue(), false);
        }
    }

    std::pair<double, bool> SignedDistanceField::DistanceToBoundary(const double x, const double y, const double z) const
    {
        return DistanceToBoundary4d(Eigen::Vector4d(x, y, z, 1.0));
    }

    std::pair<double, bool> SignedDistanceField::DistanceToBoundary3d(const Eigen::Vector3d& location) const
    {
        return DistanceToBoundary4d(Eigen::Vector4d(location.x(), location.y(), location.z(), 1.0));
    }

    std::pair<double, bool> SignedDistanceField::DistanceToBoundary4d(const Eigen::Vector4d& location) const
    {
        const auto inverse_origin_transform = distance_field_.GetInverseOriginTransform();
        const auto point_in_grid_frame = inverse_origin_transform * location;
        const auto x_size = distance_field_.GetXSize();
        const auto y_size = distance_field_.GetYSize();
        const auto z_size = distance_field_.GetZSize();
        const auto displacements = Eigen::Array3d(
                    std::min(point_in_grid_frame(0), x_size - point_in_grid_frame(0)),
                    std::min(point_in_grid_frame(1), y_size - point_in_grid_frame(1)),
                    std::min(point_in_grid_frame(2), z_size - point_in_grid_frame(2)));
        const bool point_inside = (displacements >= 0.0).all();
        const Eigen::Array3d distances = displacements.abs();
        Eigen::Array3d::Index min_index;
        distances.minCoeff(&min_index);
        return {displacements(min_index), point_inside};
    }

    std::vector<double> SignedDistanceField::GetGradient(const double x, const double y, const double z, const bool enable_edge_gradients) const
    {
        return GetGradient4d(Eigen::Vector4d(x, y, z, 1.0), enable_edge_gradients);
    }

    std::vector<double> SignedDistanceField::GetGradient3d(const Eigen::Vector3d& location, const bool enable_edge_gradients) const
    {
        const std::vector<int64_t> indices = LocationToGridIndex3d(location);
        if (indices.size() == 3)
        {
            return GetGradient(indices[0], indices[1], indices[2], enable_edge_gradients);
        }
        else
        {
            return std::vector<double>();
        }
    }

    std::vector<double> SignedDistanceField::GetGradient4d(const Eigen::Vector4d& location, const bool enable_edge_gradients) const
    {
        const std::vector<int64_t> indices = LocationToGridIndex4d(location);
        if (indices.size() == 3)
        {
            return GetGradient(indices[0], indices[1], indices[2], enable_edge_gradients);
        }
        else
        {
            return std::vector<double>();
        }
    }

    std::vector<double> SignedDistanceField::GetGradient(const VoxelGrid::GRID_INDEX& index, const bool enable_edge_gradients) const
    {
        return GetGradient(index.x, index.y, index.z, enable_edge_gradients);
    }

    std::vector<double> SignedDistanceField::GetGradient(const int64_t x_index, const int64_t y_index, const int64_t z_index, const bool enable_edge_gradients) const
    {
        // Make sure the index is inside bounds
        if ((x_index >= 0) && (y_index >= 0) && (z_index >= 0) && (x_index < GetNumXCells()) && (y_index < GetNumYCells()) && (z_index < GetNumZCells()))
        {
            // Make sure the index we're trying to query is one cell in from the edge
            if ((x_index > 0) && (y_index > 0) && (z_index > 0) && (x_index < (GetNumXCells() - 1)) && (y_index < (GetNumYCells() - 1)) && (z_index < (GetNumZCells() - 1)))
            {
                double inv_twice_resolution = 1.0 / (2.0 * GetResolution());
                double gx = (Get(x_index + 1, y_index, z_index) - Get(x_index - 1, y_index, z_index)) * inv_twice_resolution;
                double gy = (Get(x_index, y_index + 1, z_index) - Get(x_index, y_index - 1, z_index)) * inv_twice_resolution;
                double gz = (Get(x_index, y_index, z_index + 1) - Get(x_index, y_index, z_index - 1)) * inv_twice_resolution;
                return std::vector<double>{gx, gy, gz};
            }
            // If we're on the edge, handle it specially
            else if (enable_edge_gradients)
            {
                // Get the "best" indices we can use
                int64_t low_x_index = std::max((int64_t)0, x_index - 1);
                int64_t high_x_index = std::min(GetNumXCells() - 1, x_index + 1);
                int64_t low_y_index = std::max((int64_t)0, y_index - 1);
                int64_t high_y_index = std::min(GetNumYCells() - 1, y_index + 1);
                int64_t low_z_index = std::max((int64_t)0, z_index - 1);
                int64_t high_z_index = std::min(GetNumZCells() - 1, z_index + 1);
                // Compute the axis increments
                double x_increment = (high_x_index - low_x_index) * GetResolution();
                double y_increment = (high_y_index - low_y_index) * GetResolution();
                double z_increment = (high_z_index - low_z_index) * GetResolution();
                // Compute the gradients for each axis - by default these are zero
                double gx = 0.0;
                double gy = 0.0;
                double gz = 0.0;
                // Only if the increments are non-zero do we compute the gradient of an axis
                if (x_increment > 0.0)
                {
                    double inv_x_increment = 1.0 / x_increment;
                    double high_x_value = Get(high_x_index, y_index, z_index);
                    double low_x_value = Get(low_x_index, y_index, z_index);
                    // Compute the gradient
                    gx = (high_x_value - low_x_value) * inv_x_increment;
                }
                if (y_increment > 0.0)
                {
                    double inv_y_increment = 1.0 / y_increment;
                    double high_y_value = Get(x_index, high_y_index, z_index);
                    double low_y_value = Get(x_index, low_y_index, z_index);
                    // Compute the gradient
                    gy = (high_y_value - low_y_value) * inv_y_increment;
                }
                if (z_increment > 0.0)
                {
                    double inv_z_increment = 1.0 / z_increment;
                    double high_z_value = Get(x_index, y_index, high_z_index);
                    double low_z_value = Get(x_index, y_index, low_z_index);
                    // Compute the gradient
                    gz = (high_z_value - low_z_value) * inv_z_increment;
                }
                // Assemble and return the computed gradient
                return std::vector<double>{gx, gy, gz};
            }
            // Edge gradients disabled, return no gradient
            else
            {
                return std::vector<double>();
            }
        }
        // If we're out of bounds, return no gradient
        else
        {
            return std::vector<double>();
        }
    }

    Eigen::Vector3d SignedDistanceField::ProjectOutOfCollision(const double x, const double y, const double z, const double stepsize_multiplier) const
    {
        const Eigen::Vector4d result = ProjectOutOfCollision4d(Eigen::Vector4d(x, y, z, 1.0), stepsize_multiplier);
        return result.head<3>();
    }

    Eigen::Vector3d SignedDistanceField::ProjectOutOfCollisionToMinimumDistance(const double x, const double y, const double z, const double minimum_distance, const double stepsize_multiplier) const
    {
        const Eigen::Vector4d result = ProjectOutOfCollisionToMinimumDistance4d(Eigen::Vector4d(x, y, z, 1.0), minimum_distance, stepsize_multiplier);
        return result.head<3>();
    }

    Eigen::Vector3d SignedDistanceField::ProjectOutOfCollision3d(const Eigen::Vector3d& location, const double stepsize_multiplier) const
    {
        return ProjectOutOfCollision(location.x(), location.y(), location.z(), stepsize_multiplier);
    }

    Eigen::Vector3d SignedDistanceField::ProjectOutOfCollisionToMinimumDistance3d(const Eigen::Vector3d& location, const double minimum_distance, const double stepsize_multiplier) const
    {
        return ProjectOutOfCollisionToMinimumDistance(location.x(), location.y(), location.z(), minimum_distance, stepsize_multiplier);
    }

    Eigen::Vector4d SignedDistanceField::ProjectOutOfCollision4d(const Eigen::Vector4d& location, const double stepsize_multiplier) const
    {
        return ProjectOutOfCollisionToMinimumDistance4d(location, 0.0, stepsize_multiplier);
    }

    Eigen::Vector4d SignedDistanceField::ProjectOutOfCollisionToMinimumDistance4d(const Eigen::Vector4d& location, const double minimum_distance, const double stepsize_multiplier) const
    {
        // To avoid potential problems with alignment, we need to pass location by reference, so we make a local copy
        // here that we can change. https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
        Eigen::Vector4d mutable_location = location;

        // Verify that the input location is in bounds
        const auto distance_check = EstimateDistance4d(location);
        if (!distance_check.second)
        {
            std::cerr << "starting location out of bounds: " << location.transpose() << std::endl;
            std::cerr << std::endl << std::endl;
        }

        // If we are in bounds, start the projection process, otherwise return the location unchanged
        if (distance_check.second)
        {
            // TODO: make this additional margin configurable
            // Add a small collision margin to account for rounding and similar
            const double minimum_distance_with_margin = minimum_distance + GetResolution() * stepsize_multiplier * 1e-3;
            const double max_stepsize = GetResolution() * stepsize_multiplier;
            const bool enable_edge_gradients = true;

            double sdf_dist = EstimateDistance4d(mutable_location).first;
            while (sdf_dist <= minimum_distance)
            {
                const std::vector<double> gradient = GetGradient4d(mutable_location, enable_edge_gradients);
                assert(gradient.size() == 3);
                const Eigen::Vector4d grad_eigen(gradient[0], gradient[1], gradient[2], 0.0);
                assert(grad_eigen.norm() > GetResolution() * 0.25); // Sanity check
                // Don't step any farther than is needed
                const double step_distance = std::min(max_stepsize, minimum_distance_with_margin - sdf_dist);
                const Eigen::Vector4d next_location = mutable_location + grad_eigen.normalized() * step_distance;

                // Ensure that we are still operating in the valid range of the SDF
                const auto distance_check = EstimateDistance4d(next_location);
                if (!distance_check.second)
                {
                    std::cerr << "starting location: " << location.transpose() << std::endl
                              << "current location:  " << mutable_location.transpose() << std::endl
                              << "Current gradient:  " << grad_eigen.transpose() << std::endl
                              << "Out of bounds loc: " << next_location.transpose() << std::endl;
                    std::cerr << std::endl << std::endl;
                }
                mutable_location = next_location;
                sdf_dist = distance_check.first;
            }
        }
        return mutable_location;
    }

    Eigen::Vector3d SignedDistanceField::ProjectIntoValidVolume(const double x, const double y, const double z) const
    {
        const Eigen::Vector4d result = ProjectIntoValidVolume4d(Eigen::Vector4d(x, y, z, 1.0));
        return result.head<3>();
    }

    Eigen::Vector3d SignedDistanceField::ProjectIntoValidVolumeToMinimumDistance(const double x, const double y, const double z, const double minimum_distance) const
    {
        const Eigen::Vector4d result = ProjectIntoValidVolumeToMinimumDistance4d(Eigen::Vector4d(x, y, z, 1.0), minimum_distance);
        return result.head<3>();
    }

    Eigen::Vector3d SignedDistanceField::ProjectIntoValidVolume3d(const Eigen::Vector3d& location) const
    {
        return ProjectIntoValidVolume(location.x(), location.y(), location.z());
    }

    Eigen::Vector3d SignedDistanceField::ProjectIntoValidVolumeToMinimumDistance3d(const Eigen::Vector3d& location, const double minimum_distance) const
    {
        return ProjectIntoValidVolumeToMinimumDistance(location.x(), location.y(), location.z(), minimum_distance);
    }

    Eigen::Vector4d SignedDistanceField::ProjectIntoValidVolume4d(const Eigen::Vector4d& location) const
    {
        return ProjectIntoValidVolumeToMinimumDistance4d(location, 0.0);
    }

    Eigen::Vector4d SignedDistanceField::ProjectIntoValidVolumeToMinimumDistance4d(const Eigen::Vector4d& location, const double minimum_distance) const
    {
        const auto inverse_origin_transform = distance_field_.GetInverseOriginTransform();
        const auto point_in_grid_frame = inverse_origin_transform * location;
        const auto x_size = distance_field_.GetXSize();
        const auto y_size = distance_field_.GetYSize();
        const auto z_size = distance_field_.GetZSize();
        // TODO: make this additional margin configurable
        const double minimum_distance_with_margin = minimum_distance + GetResolution() * 1e-4;
        const double x = arc_helpers::ClampValue(point_in_grid_frame(0), minimum_distance_with_margin, x_size - minimum_distance_with_margin);
        const double y = arc_helpers::ClampValue(point_in_grid_frame(1), minimum_distance_with_margin, y_size - minimum_distance_with_margin);
        const double z = arc_helpers::ClampValue(point_in_grid_frame(2), minimum_distance_with_margin, z_size - minimum_distance_with_margin);
        // To avoid numerical problems, only return a modified location if we actually had to change something
        bool change_made = false;
        change_made |= (x != point_in_grid_frame(0));
        change_made |= (y != point_in_grid_frame(1));
        change_made |= (z != point_in_grid_frame(2));
        if (change_made)
        {
            return GetOriginTransform() * Eigen::Vector4d(x, y, z, 1.0);
        }
        else
        {
            return location;
        }
    }

    const Eigen::Isometry3d& SignedDistanceField::GetOriginTransform() const
    {
        return distance_field_.GetOriginTransform();
    }

    const Eigen::Isometry3d& SignedDistanceField::GetInverseOriginTransform() const
    {
        return distance_field_.GetInverseOriginTransform();
    }

    std::string SignedDistanceField::GetFrame() const
    {
        return frame_;
    }

    std::vector<int64_t> SignedDistanceField::LocationToGridIndex3d(const Eigen::Vector3d& location) const
    {
        return distance_field_.LocationToGridIndex3d(location);
    }

    std::vector<int64_t> SignedDistanceField::LocationToGridIndex4d(const Eigen::Vector4d& location) const
    {
        return distance_field_.LocationToGridIndex4d(location);
    }

    std::vector<int64_t> SignedDistanceField::LocationToGridIndex(const double x, const double y, const double z) const
    {
        return distance_field_.LocationToGridIndex(x, y, z);
    }

    std::vector<double> SignedDistanceField::GridIndexToLocation(const VoxelGrid::GRID_INDEX& index) const
    {
        return distance_field_.GridIndexToLocation(index);
    }

    std::vector<double> SignedDistanceField::GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        return distance_field_.GridIndexToLocation(x_index, y_index, z_index);
    }

    bool SignedDistanceField::SaveToFile(const std::string& filepath)
    {
        // Convert to message representation
        sdf_tools::SDF message_rep = GetMessageRepresentation();
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

    bool SignedDistanceField::LoadFromFile(const std::string &filepath)
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
            sdf_tools::SDF new_message;
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

    sdf_tools::SDF SignedDistanceField::GetMessageRepresentation()
    {
        sdf_tools::SDF message_rep;
        // Populate message
        message_rep.header.frame_id = frame_;
        const Eigen::Isometry3d& origin_transform = distance_field_.GetOriginTransform();
        message_rep.origin_transform.translation.x = origin_transform.translation().x();
        message_rep.origin_transform.translation.y = origin_transform.translation().y();
        message_rep.origin_transform.translation.z = origin_transform.translation().z();
        const Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
        message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
        message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
        message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
        message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
        message_rep.dimensions.x = distance_field_.GetXSize();
        message_rep.dimensions.y = distance_field_.GetYSize();
        message_rep.dimensions.z = distance_field_.GetZSize();
        message_rep.sdf_cell_size = GetResolution();
        message_rep.OOB_value = distance_field_.GetDefaultValue();
        message_rep.initialized = initialized_;
        message_rep.locked = locked_;
        const std::vector<float>& raw_data = distance_field_.GetRawData();
        std::vector<uint8_t> binary_data = GetInternalBinaryRepresentation(raw_data);
        message_rep.data = ZlibHelpers::CompressBytes(binary_data);
        return message_rep;
    }

    bool SignedDistanceField::LoadFromMessageRepresentation(const SDF& message)
    {
        // Make a new voxel grid inside
        Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
        Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
        Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
        VoxelGrid::VoxelGrid<float> new_field(origin_transform, message.sdf_cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, message.OOB_value);
        // Unpack the binary data
        std::vector<uint8_t> binary_data = ZlibHelpers::DecompressBytes(message.data);
        std::vector<float> unpacked = UnpackFieldFromBinaryRepresentation(binary_data);
        if (unpacked.empty())
        {
            std::cerr << "Unpack returned an empty SDF" << std::endl;
            return false;
        }
        bool success = new_field.SetRawData(unpacked);
        if (!success)
        {
            std::cerr << "Unable to set internal representation of the SDF" << std::endl;
            return false;
        }
        // Set it
        distance_field_ = new_field;
        frame_ = message.header.frame_id;
        initialized_ = message.initialized;
        locked_ = message.locked;
        return true;
    }

    visualization_msgs::Marker SignedDistanceField::ExportForDisplay(const float alpha) const
    {
        // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
        visualization_msgs::Marker display_rep;
        // Populate the header
        display_rep.header.frame_id = frame_;
        // Populate the options
        display_rep.ns = "sdf_display";
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
        double min_distance = 0.0;
        double max_distance = 0.0;
        for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
                {
                    // Update minimum/maximum distance variables
                    float distance = Get(x_index, y_index, z_index);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                    if (distance > max_distance)
                    {
                        max_distance = distance;
                    }
                    // Convert SDF indices into a real-world location
                    std::vector<double> location = distance_field_.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                }
            }
        }
        // Add colors for all the cells of the SDF to the message
        for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
                {
                    // Update minimum/maximum distance variables
                    float distance = Get(x_index, y_index, z_index);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    if (distance > 0.0)
                    {
                        new_color.b = 0.0;
                        new_color.g = (std::fabs(distance / max_distance) * 0.8) + 0.2;
                        new_color.r = 0.0;
                    }
                    else if (distance < 0.0)
                    {
                        new_color.b = 0.0;
                        new_color.g = 0.0;
                        new_color.r = (std::fabs(distance / min_distance) * 0.8) + 0.2;
                    }
                    else
                    {
                        new_color.b = 1.0;
                        new_color.g = 0.0;
                        new_color.r = 0.0;
                    }
                    display_rep.colors.push_back(new_color);
                }
            }
        }
        return display_rep;
    }

    visualization_msgs::Marker SignedDistanceField::ExportForDisplayCollisionOnly(const float alpha) const
    {
        // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
        visualization_msgs::Marker display_rep;
        // Populate the header
        display_rep.header.frame_id = frame_;
        // Populate the options
        display_rep.ns = "sdf_display";
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
        for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
                {
                    // Update minimum/maximum distance variables
                    float distance = Get(x_index, y_index, z_index);
                    if (distance <= 0.0)
                    {
                        // Convert SDF indices into a real-world location
                        std::vector<double> location = distance_field_.GridIndexToLocation(x_index, y_index, z_index);
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
                }
            }
        }
        return display_rep;
    }

    visualization_msgs::Marker SignedDistanceField::ExportForDebug(const float alpha) const
    {
        // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
        visualization_msgs::Marker display_rep;
        // Populate the header
        display_rep.header.frame_id = frame_;
        // Populate the options
        display_rep.ns = "sdf_display";
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
        for (int64_t x_index = 0; x_index < distance_field_.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < distance_field_.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < distance_field_.GetNumZCells(); z_index++)
                {
                    // Convert SDF indices into a real-world location
                    std::vector<double> location = distance_field_.GridIndexToLocation(x_index, y_index, z_index);
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    // Color it
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 1.0;
                    new_color.r = 1.0;
                    display_rep.colors.push_back(new_color);
                }
            }
        }
        return display_rep;
    }

    VoxelGrid::VoxelGrid<Eigen::Vector3d> SignedDistanceField::ComputeLocalMaximaMap() const
    {
        VoxelGrid::VoxelGrid<Eigen::Vector3d> watershed_map(GetOriginTransform(), GetResolution(), GetXSize(), GetYSize(), GetZSize(), Eigen::Vector3d(-INFINITY, -INFINITY, -INFINITY));
        for (int64_t x_idx = 0; x_idx < watershed_map.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < watershed_map.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < watershed_map.GetNumZCells(); z_idx++)
                {
                    // We use an "unsafe" function here because we know all the indices we provide it will be safe
                    FollowGradientsToLocalMaximaUnsafe(watershed_map, x_idx, y_idx, z_idx);
                }
            }
        }
        return watershed_map;
    }

    bool SignedDistanceField::GradientIsEffectiveFlat(const Eigen::Vector3d& gradient) const
    {
        // A gradient is at a local maxima if the absolute value of all components (x,y,z) are less than 1/2 SDF resolution
        double half_resolution = GetResolution() * 0.5;
        if (std::fabs(gradient.x()) <= half_resolution && std::fabs(gradient.y()) <= half_resolution && std::fabs(gradient.z()) <= half_resolution)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    VoxelGrid::GRID_INDEX SignedDistanceField::GetNextFromGradient(const VoxelGrid::GRID_INDEX& index, const Eigen::Vector3d& gradient) const
    {
        // Given the gradient, pick the "best fit" of the 26 neighboring points
        VoxelGrid::GRID_INDEX next_index = index;
        double half_resolution = GetResolution() * 0.5;
        if (gradient.x() > half_resolution)
        {
            next_index.x++;
        }
        else if (gradient.x() < -half_resolution)
        {
            next_index.x--;
        }
        if (gradient.y() > half_resolution)
        {
            next_index.y++;
        }
        else if (gradient.y() < -half_resolution)
        {
            next_index.y--;
        }
        if (gradient.z() > half_resolution)
        {
            next_index.z++;
        }
        else if (gradient.z() < -half_resolution)
        {
            next_index.z--;
        }
        return next_index;
    }


}
