#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Geometry>
#include <stdexcept>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/serialization_eigen.hpp>
#include <arc_utilities/eigen_helpers.hpp>

#ifndef VOXEL_GRID_HPP
#define VOXEL_GRID_HPP

namespace VoxelGrid
{
    struct GRID_INDEX
    {
        int64_t x;
        int64_t y;
        int64_t z;

        GRID_INDEX() : x(-1), y(-1), z(-1) {}

        GRID_INDEX(const int64_t in_x, const int64_t in_y, const int64_t in_z) : x(in_x), y(in_y), z(in_z) {}

        bool operator==(const GRID_INDEX& other) const
        {
            return (x == other.x && y == other.y && z == other.z);
        }
    };

    template<typename T, typename Allocator=std::allocator<T>>
    class VoxelGrid
    {
    protected:

        Eigen::Isometry3d origin_transform_;
        Eigen::Isometry3d inverse_origin_transform_;
        T default_value_;
        T oob_value_;
        std::vector<T, Allocator> data_;
        double cell_x_size_;
        double cell_y_size_;
        double cell_z_size_;
        double inv_cell_x_size_;
        double inv_cell_y_size_;
        double inv_cell_z_size_;
        double x_size_;
        double y_size_;
        double z_size_;
        int64_t stride1_;
        int64_t stride2_;
        int64_t num_x_cells_;
        int64_t num_y_cells_;
        int64_t num_z_cells_;
        bool initialized_;

        inline int64_t GetDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }

        inline void SetContents(const T& value)
        {
            data_.clear();
            data_.resize(num_x_cells_ * num_y_cells_ * num_z_cells_, value);
        }

        inline void SafetyCheckSizes(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size,const double y_size, const double z_size) const
        {
            if (cell_x_size <= 0.0)
            {
                throw std::invalid_argument("cell_x_size must be positive and non-zero");
            }
            if (std::isnan(cell_x_size))
            {
                throw std::invalid_argument("cell_x_size must not be NaN");
            }
            if (std::isinf(cell_x_size) != 0)
            {
                throw std::invalid_argument("cell_x_size must not be INF");
            }
            if (cell_y_size <= 0.0)
            {
                throw std::invalid_argument("cell_y_size must be positive and non-zero");
            }
            if (std::isnan(cell_y_size))
            {
                throw std::invalid_argument("cell_y_size must not be NaN");
            }
            if (std::isinf(cell_y_size) != 0)
            {
                throw std::invalid_argument("cell_y_size must not be INF");
            }
            if (cell_z_size <= 0.0)
            {
                throw std::invalid_argument("cell_z_size must be positive and non-zero");
            }
            if (std::isnan(cell_z_size))
            {
                throw std::invalid_argument("cell_z_size must not be NaN");
            }
            if (std::isinf(cell_z_size) != 0)
            {
                throw std::invalid_argument("cell_z_size must not be INF");
            }
            if (x_size <= 0.0)
            {
                throw std::invalid_argument("x_size must be positive and non-zero");
            }
            if (y_size <= 0.0)
            {
                throw std::invalid_argument("y_size must be positive and non-zero");
            }
            if (z_size <= 0.0)
            {
                throw std::invalid_argument("z_size must be positive and non-zero");
            }
            if (std::isnan(x_size))
            {
                throw std::invalid_argument("x_size must not be NaN");
            }
            if (std::isnan(y_size))
            {
                throw std::invalid_argument("y_size must not be NaN");
            }
            if (std::isnan(z_size))
            {
                throw std::invalid_argument("z_size must not be NaN");
            }
            if (std::isinf(x_size) != 0)
            {
                throw std::invalid_argument("x_size must not be INF");
            }
            if (std::isinf(y_size) != 0)
            {
                throw std::invalid_argument("y_size must not be INF");
            }
            if (std::isinf(z_size) != 0)
            {
                throw std::invalid_argument("z_size must not be INF");
            }
        }

        inline void SafetyCheckSizes(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells,const int64_t num_y_cells, const int64_t num_z_cells) const
        {
            if (cell_x_size <= 0.0)
            {
                throw std::invalid_argument("cell_x_size must be positive and non-zero");
            }
            if (std::isnan(cell_x_size))
            {
                throw std::invalid_argument("cell_x_size must not be NaN");
            }
            if (std::isinf(cell_x_size) != 0)
            {
                throw std::invalid_argument("cell_x_size must not be INF");
            }
            if (cell_y_size <= 0.0)
            {
                throw std::invalid_argument("cell_y_size must be positive and non-zero");
            }
            if (std::isnan(cell_y_size))
            {
                throw std::invalid_argument("cell_y_size must not be NaN");
            }
            if (std::isinf(cell_y_size) != 0)
            {
                throw std::invalid_argument("cell_y_size must not be INF");
            }
            if (cell_z_size <= 0.0)
            {
                throw std::invalid_argument("cell_z_size must be positive and non-zero");
            }
            if (std::isnan(cell_z_size))
            {
                throw std::invalid_argument("cell_z_size must not be NaN");
            }
            if (std::isinf(cell_z_size) != 0)
            {
                throw std::invalid_argument("cell_z_size must not be INF");
            }
            if (num_x_cells <= 0)
            {
                throw std::invalid_argument("num_x_cells must be positive and non-zero");
            }
            if (num_y_cells <= 0)
            {
                throw std::invalid_argument("num_y_cells must be positive and non-zero");
            }
            if (num_z_cells <= 0)
            {
                throw std::invalid_argument("num_z_cells must be positive and non-zero");
            }
        }

        inline void CoreInitialize(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            inv_cell_x_size_ = 1.0 / cell_x_size_;
            inv_cell_y_size_ = 1.0 / cell_y_size_;
            inv_cell_z_size_ = 1.0 / cell_z_size_;
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            default_value_ = default_value;
            oob_value_ = oob_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        inline static uint64_t Serialize(const VoxelGrid<T, Allocator>& grid, std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer)
        {
            return grid.SerializeSelf(buffer, value_serializer);
        }

        inline static std::pair<VoxelGrid<T, Allocator>, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
        {
            VoxelGrid<T, Allocator> temp_grid;
            const uint64_t bytes_read = temp_grid.DeserializeSelf(buffer, current, value_deserializer);
            return std::make_pair(temp_grid, bytes_read);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_size, const double x_size, const double y_size, double const z_size, const T& default_value)
        {
            Initialize(origin_transform, cell_size, cell_size, cell_size, x_size, y_size, z_size, default_value, default_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_size, const double x_size, const double y_size, const double z_size, const T& default_value, const T& oob_value)
        {
            Initialize(origin_transform, cell_size, cell_size, cell_size, x_size, y_size, z_size, default_value, oob_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, double const z_size, const T& default_value)
        {
            Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size, default_value, default_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, const double z_size, const T& default_value, const T& oob_value)
        {
            Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size, default_value, oob_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value)
        {
            Initialize(origin_transform, cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells, default_value, default_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            Initialize(origin_transform, cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value)
        {
            Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, default_value);
        }

        VoxelGrid(const Eigen::Isometry3d& origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
        }

        VoxelGrid(const double cell_size, const double x_size, const double y_size, const double z_size, const T& default_value)
        {
            Initialize(cell_size, cell_size, cell_size, x_size, y_size, z_size, default_value, default_value);
        }

        VoxelGrid(const double cell_size, const double x_size, const double y_size, const double z_size, const T& default_value, const T& oob_value)
        {
            Initialize(cell_size, cell_size, cell_size, x_size, y_size, z_size, default_value, oob_value);
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, const double z_size, const T& default_value)
        {
            Initialize(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size, default_value, default_value);
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, const double z_size, const T& default_value, const T& oob_value)
        {
            Initialize(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size, default_value, oob_value);
        }

        VoxelGrid(const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value)
        {
            Initialize(cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells, default_value, default_value);
        }

        VoxelGrid(const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            Initialize(cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value)
        {
            Initialize(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, default_value);
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            Initialize(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
        }

        VoxelGrid()
        {
            origin_transform_ = Eigen::Isometry3d::Identity();
            inverse_origin_transform_ = origin_transform_.inverse();
            arc_helpers::RequireAlignment(origin_transform_, 16u);
            arc_helpers::RequireAlignment(inverse_origin_transform_, 16u);
            cell_x_size_ = 0.0;
            cell_y_size_ = 0.0;
            cell_z_size_ = 0.0;
            inv_cell_x_size_ = 0.0;
            inv_cell_y_size_ = 0.0;
            inv_cell_z_size_ = 0.0;
            x_size_ = 0.0;
            y_size_ = 0.0;
            z_size_ = 0.0;
            num_x_cells_ = 0;
            num_y_cells_ = 0;
            num_z_cells_ = 0;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            data_.clear();
            initialized_ = false;
        }

        inline void Initialize(const Eigen::Isometry3d& origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, double const z_size, const T& default_value, const T& oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size);
            int64_t num_x_cells = (int64_t)(ceil(fabs(x_size) / fabs(cell_x_size)));
            int64_t num_y_cells = (int64_t)(ceil(fabs(y_size) / fabs(cell_y_size)));
            int64_t num_z_cells = (int64_t)(ceil(fabs(z_size) / fabs(cell_z_size)));
            Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
        }

        inline void Initialize(const Eigen::Isometry3d& origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            CoreInitialize(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            arc_helpers::RequireAlignment(origin_transform_, 16u);
            arc_helpers::RequireAlignment(inverse_origin_transform_, 16u);
            initialized_ = true;
        }

        inline void Initialize(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, double const z_size, const T& default_value, const T& oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size);
            int64_t num_x_cells = (int64_t)(ceil(fabs(x_size) / fabs(cell_x_size)));
            int64_t num_y_cells = (int64_t)(ceil(fabs(y_size) / fabs(cell_y_size)));
            int64_t num_z_cells = (int64_t)(ceil(fabs(z_size) / fabs(cell_z_size)));
            Initialize(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
        }

        inline void Initialize(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T& default_value, const T& oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            CoreInitialize(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
            const Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            const Eigen::Isometry3d origin_transform = origin_translation * Eigen::Quaterniond::Identity();
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            arc_helpers::RequireAlignment(origin_transform_, 16u);
            arc_helpers::RequireAlignment(inverse_origin_transform_, 16u);
            initialized_ = true;
        }

        inline uint64_t SerializeSelf(std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer) const
        {
            const uint64_t start_buffer_size = buffer.size();
            // Serialize the initialized
            arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)initialized_, buffer);
            // Serialize the transforms
            arc_utilities::SerializeEigenType<Eigen::Isometry3d>(origin_transform_, buffer);
            arc_utilities::SerializeEigenType<Eigen::Isometry3d>(inverse_origin_transform_, buffer);
            // Serialize the data
            arc_utilities::SerializeVector<T, Allocator>(data_, buffer, value_serializer);
            // Serialize the cell sizes
            arc_utilities::SerializeFixedSizePOD<double>(cell_x_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(cell_y_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(cell_z_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(inv_cell_x_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(inv_cell_y_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(inv_cell_z_size_, buffer);
            // Serialize the grid sizes
            arc_utilities::SerializeFixedSizePOD<double>(x_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(y_size_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(z_size_, buffer);
            // Serialize the control/bounds values
            arc_utilities::SerializeFixedSizePOD<int64_t>(stride1_, buffer);
            arc_utilities::SerializeFixedSizePOD<int64_t>(stride2_, buffer);
            arc_utilities::SerializeFixedSizePOD<int64_t>(num_x_cells_, buffer);
            arc_utilities::SerializeFixedSizePOD<int64_t>(num_y_cells_, buffer);
            arc_utilities::SerializeFixedSizePOD<int64_t>(num_z_cells_, buffer);
            // Serialize the default value
            value_serializer(default_value_, buffer);
            // Serialize the OOB value
            value_serializer(oob_value_, buffer);
            // Figure out how many bytes were written
            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;
            return bytes_written;
        }

        inline uint64_t DeserializeSelf(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
        {
            uint64_t current_position = current;
            // Deserialize the initialized
            const std::pair<uint8_t, uint64_t> initialized_deserialized = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
            initialized_ = (bool)initialized_deserialized.first;
            current_position += initialized_deserialized.second;
            // Deserialize the transforms
            const std::pair<Eigen::Isometry3d, uint64_t> origin_transform_deserialized = arc_utilities::DeserializeEigenType<Eigen::Isometry3d>(buffer, current_position);
            origin_transform_ = origin_transform_deserialized.first;
            current_position += origin_transform_deserialized.second;
            const std::pair<Eigen::Isometry3d, uint64_t> inverse_origin_transform_deserialized = arc_utilities::DeserializeEigenType<Eigen::Isometry3d>(buffer, current_position);
            inverse_origin_transform_ = inverse_origin_transform_deserialized.first;
            current_position += inverse_origin_transform_deserialized.second;
            // Deserialize the data
            const std::pair<std::vector<T, Allocator>, uint64_t> data_deserialized = arc_utilities::DeserializeVector<T, Allocator>(buffer, current_position, value_deserializer);
            data_ = data_deserialized.first;
            current_position += data_deserialized.second;
            // Deserialize the cell sizes
            const std::pair<double, uint64_t> cell_x_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            cell_x_size_ = cell_x_size_deserialized.first;
            current_position += cell_x_size_deserialized.second;
            const std::pair<double, uint64_t> cell_y_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            cell_y_size_ = cell_y_size_deserialized.first;
            current_position += cell_y_size_deserialized.second;
            const std::pair<double, uint64_t> cell_z_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            cell_z_size_ = cell_z_size_deserialized.first;
            current_position += cell_z_size_deserialized.second;
            const std::pair<double, uint64_t> inv_cell_x_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            inv_cell_x_size_ = inv_cell_x_size_deserialized.first;
            current_position += inv_cell_x_size_deserialized.second;
            const std::pair<double, uint64_t> inv_cell_y_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            inv_cell_y_size_ = inv_cell_y_size_deserialized.first;
            current_position += inv_cell_y_size_deserialized.second;
            const std::pair<double, uint64_t> inv_cell_z_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            inv_cell_z_size_ = inv_cell_z_size_deserialized.first;
            current_position += inv_cell_z_size_deserialized.second;
            // Deserialize the grid sizes
            const std::pair<double, uint64_t> x_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            x_size_ = x_size_deserialized.first;
            current_position += x_size_deserialized.second;
            const std::pair<double, uint64_t> y_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            y_size_ = y_size_deserialized.first;
            current_position += y_size_deserialized.second;
            const std::pair<double, uint64_t> z_size_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
            z_size_ = z_size_deserialized.first;
            current_position += z_size_deserialized.second;
            // Deserialize the control/bounds values
            const std::pair<int64_t, uint64_t> stride1_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            stride1_ = stride1_deserialized.first;
            current_position += stride1_deserialized.second;
            const std::pair<int64_t, uint64_t> stride2_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            stride2_ = stride2_deserialized.first;
            current_position += stride2_deserialized.second;
            const std::pair<int64_t, uint64_t> num_x_cells_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            num_x_cells_ = num_x_cells_deserialized.first;
            current_position += num_x_cells_deserialized.second;
            const std::pair<int64_t, uint64_t> num_y_cells_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            num_y_cells_ = num_y_cells_deserialized.first;
            current_position += num_y_cells_deserialized.second;
            const std::pair<int64_t, uint64_t> num_z_cells_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            num_z_cells_ = num_z_cells_deserialized.first;
            current_position += num_z_cells_deserialized.second;
            // Deserialize the default value
            const std::pair<T, uint64_t> default_value_deserialized = value_deserializer(buffer, current_position);
            default_value_ = default_value_deserialized.first;
            current_position += default_value_deserialized.second;
            // Deserialize the OOB value
            const std::pair<T, uint64_t> oob_value_deserialized = value_deserializer(buffer, current_position);
            oob_value_ = oob_value_deserialized.first;
            current_position += oob_value_deserialized.second;
            // Figure out how many bytes were read
            const uint64_t bytes_read = current_position - current;
            return bytes_read;
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline void ResetWithDefault()
        {
            SetContents(default_value_);
        }

        inline void ResetWithNewValue(const T& new_value)
        {
            SetContents(new_value);
        }

        inline void ResetWithNewDefault(const T& new_default)
        {
            default_value_ = new_default;
            SetContents(default_value_);
        }

        inline bool IndexInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            if (x_index >= 0 && y_index >= 0 && z_index >= 0 && x_index < num_x_cells_ && y_index < num_y_cells_ && z_index < num_z_cells_)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline std::pair<const T&, bool> GetImmutable3d(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex3d(location);
            if (indices.size() == 3)
            {
                return GetImmutable(indices[0], indices[1], indices[2]);
            }
            else
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
        }

        inline std::pair<const T&, bool> GetImmutable4d(const Eigen::Vector4d& location) const
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex4d(location);
            if (indices.size() == 3)
            {
                return GetImmutable(indices[0], indices[1], indices[2]);
            }
            else
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
        }

        inline std::pair<const T&, bool> GetImmutable(const double x, const double y, const double z) const
        {
            const Eigen::Vector4d location(x, y, z, 1.0);
            return GetImmutable4d(location);
        }

        inline std::pair<const T&, bool> GetImmutable(const GRID_INDEX& index) const
        {
            return GetImmutable(index.x, index.y, index.z);
        }

        inline std::pair<const T&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            assert(initialized_);
            if (IndexInBounds(x_index, y_index, z_index))
            {
                const int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < (int64_t)data_.size());
                return std::pair<const T&, bool>(data_[data_index], true);
            }
            else
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
        }

        inline std::pair<T&, bool> GetMutable3d(const Eigen::Vector3d& location)
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex3d(location);
            if (indices.size() == 3)
            {
                return GetMutable(indices[0], indices[1], indices[2]);
            }
            else
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
        }

        inline std::pair<T&, bool> GetMutable4d(const Eigen::Vector4d& location)
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex4d(location);
            if (indices.size() == 3)
            {
                return GetMutable(indices[0], indices[1], indices[2]);
            }
            else
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
        }

        inline std::pair<T&, bool> GetMutable(const double x, const double y, const double z)
        {
            const Eigen::Vector4d location(x, y, z, 1.0);
            return GetMutable4d(location);
        }

        inline std::pair<T&, bool> GetMutable(const GRID_INDEX& index)
        {
            return GetMutable(index.x, index.y, index.z);
        }

        inline std::pair<T&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            assert(initialized_);
            if (IndexInBounds(x_index, y_index, z_index))
            {
                const int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < (int64_t)data_.size());
                return std::pair<T&, bool>(data_[data_index], true);
            }
            else
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
        }

        inline bool SetValue3d(const Eigen::Vector3d& location, const T& value)
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex3d(location);
            if (indices.size() == 3)
            {
                return SetValue(indices[0], indices[1], indices[2], value);
            }
            else
            {
                return false;
            }
        }

        inline bool SetValue4d(const Eigen::Vector4d& location, const T& value)
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex4d(location);
            if (indices.size() == 3)
            {
                return SetValue(indices[0], indices[1], indices[2], value);
            }
            else
            {
                return false;
            }
        }

        inline bool SetValue(const double x, const double y, const double z, const T& value)
        {
            const Eigen::Vector4d location(x, y, z, 1.0);
            return SetValue4d(location, value);
        }

        inline bool SetValue(const GRID_INDEX& index, const T& value)
        {
            return SetValue(index.x, index.y, index.z, value);
        }

        inline bool SetValue(const int64_t x_index, const int64_t y_index, const int64_t z_index, const T& value)
        {
            assert(initialized_);
            if (IndexInBounds(x_index, y_index, z_index))
            {
                const int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < (int64_t)data_.size());
                data_[data_index] = value;
                return true;
            }
            else
            {
                return false;
            }
        }

        inline bool SetValue3d(const Eigen::Vector3d& location, T&& value)
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex3d(location);
            if (indices.size() == 3)
            {
                return SetValue(indices[0], indices[1], indices[2], value);
            }
            else
            {
                return false;
            }
        }

        inline bool SetValue4d(const Eigen::Vector4d& location, T&& value)
        {
            assert(initialized_);
            const std::vector<int64_t> indices = LocationToGridIndex4d(location);
            if (indices.size() == 3)
            {
                return SetValue(indices[0], indices[1], indices[2], value);
            }
            else
            {
                return false;
            }
        }

        inline bool SetValue(const double x, const double y, const double z, T&& value)
        {
            const Eigen::Vector4d location(x, y, z, 1.0);
            return SetValue4d(location, value);
        }

        inline bool SetValue(const GRID_INDEX& index, T&& value)
        {
            return SetValue(index.x, index.y, index.z, value);
        }

        inline bool SetValue(const int64_t x_index, const int64_t y_index, const int64_t z_index, T&& value)
        {
            assert(initialized_);
            if (IndexInBounds(x_index, y_index, z_index))
            {
                const int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < (int64_t)data_.size());
                data_[data_index] = value;
                return true;
            }
            else
            {
                return false;
            }
        }

        inline double GetXSize() const
        {
            return x_size_;
        }

        inline double GetYSize() const
        {
            return y_size_;
        }

        inline double GetZSize() const
        {
            return z_size_;
        }

        inline std::vector<double> GetCellSizes() const
        {
            return std::vector<double>{cell_x_size_, cell_y_size_, cell_z_size_};
        }

        inline T GetDefaultValue() const
        {
            return default_value_;
        }

        inline T GetOOBValue() const
        {
            return oob_value_;
        }

        inline void SetDefaultValue(const T& default_value)
        {
            default_value_ = default_value;
        }

        inline void SetOOBValue(const T& oob_value)
        {
            oob_value_ = oob_value;
        }

        inline int64_t GetNumXCells() const
        {
            return num_x_cells_;
        }

        inline int64_t GetNumYCells() const
        {
            return num_y_cells_;
        }

        inline int64_t GetNumZCells() const
        {
            return num_z_cells_;
        }

        inline const Eigen::Isometry3d& GetOriginTransform() const
        {
            return origin_transform_;
        }

        inline const Eigen::Isometry3d& GetInverseOriginTransform() const
        {
            return inverse_origin_transform_;
        }

        inline std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const
        {
            const Eigen::Vector4d point(x, y, z, 1.0);
            return LocationToGridIndex4d(point);
        }

        inline std::vector<int64_t> LocationToGridIndex3d(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            const Eigen::Vector3d point_in_grid_frame = inverse_origin_transform_ * location;
            const int64_t x_cell = (int64_t)(point_in_grid_frame.x() * inv_cell_x_size_);
            const int64_t y_cell = (int64_t)(point_in_grid_frame.y() * inv_cell_y_size_);
            const int64_t z_cell = (int64_t)(point_in_grid_frame.z() * inv_cell_z_size_);
            if (IndexInBounds(x_cell, y_cell, z_cell))
            {
                return std::vector<int64_t>{x_cell, y_cell, z_cell};
            }
            else
            {
                return std::vector<int64_t>();
            }
        }

        inline std::vector<int64_t> LocationToGridIndex4d(const Eigen::Vector4d& location) const
        {
            assert(initialized_);
            const Eigen::Vector4d point_in_grid_frame = inverse_origin_transform_ * location;
            const int64_t x_cell = (int64_t)(point_in_grid_frame(0) * inv_cell_x_size_);
            const int64_t y_cell = (int64_t)(point_in_grid_frame(1) * inv_cell_y_size_);
            const int64_t z_cell = (int64_t)(point_in_grid_frame(2) * inv_cell_z_size_);
            if (IndexInBounds(x_cell, y_cell, z_cell))
            {
                return std::vector<int64_t>{x_cell, y_cell, z_cell};
            }
            else
            {
                return std::vector<int64_t>();
            }
        }

        inline std::vector<double> GridIndexToLocation(const GRID_INDEX& index) const
        {
            return GridIndexToLocation(index.x, index.y, index.z);
        }

        inline std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            assert(initialized_);
            if (IndexInBounds(x_index, y_index, z_index))
            {
                const Eigen::Vector3d point_in_grid_frame(cell_x_size_ * ((double)x_index + 0.5), cell_y_size_ * ((double)y_index + 0.5), cell_z_size_ * ((double)z_index + 0.5));
                const Eigen::Vector3d point = origin_transform_ * point_in_grid_frame;
                return std::vector<double>{point.x(), point.y(), point.z()};
            }
            else
            {
                return std::vector<double>();
            }
        }

        inline const std::vector<T>& GetRawData() const
        {
            return data_;
        }

        inline std::vector<T> CopyRawData() const
        {
            return data_;
        }

        inline bool SetRawData(std::vector<T>& data)
        {
            assert(initialized_);
            const int64_t expected_length = num_x_cells_ * num_y_cells_ * num_z_cells_;
            if ((int64_t)data.size() == expected_length)
            {
                data_ = data;
                return true;
            }
            else
            {
                std::cerr << "Failed to load internal data - expected " << expected_length << " got " << data.size() << std::endl;
                return false;
            }
        }

        inline uint64_t HashDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }
    };
}

namespace std
{
    template <>
    struct hash<VoxelGrid::GRID_INDEX>
    {
        std::size_t operator()(const VoxelGrid::GRID_INDEX& index) const
        {
            using std::size_t;
            using std::hash;
            return ((std::hash<int64_t>()(index.x) ^ (std::hash<int64_t>()(index.y) << 1) >> 1) ^ (std::hash<int64_t>()(index.z) << 1));
        }
    };
}

inline std::ostream& operator<<(std::ostream& strm, const VoxelGrid::GRID_INDEX& index)
{
    strm << "GridIndex: " << index.x << "," << index.y << "," << index.z;
    return strm;
}

#endif // VOXEL_GRID_HPP
