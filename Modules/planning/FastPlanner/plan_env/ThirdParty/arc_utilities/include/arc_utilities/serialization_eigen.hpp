#ifndef SERIALIZATION_EIGEN_HPP
#define SERIALIZATION_EIGEN_HPP

#include "arc_utilities/serialization.hpp"
#include "arc_utilities/eigen_helpers.hpp"

namespace arc_utilities
{
    ////////////////////////////////////////////////////////////////////////////
    // Serialization/Deserialization functions
    ////////////////////////////////////////////////////////////////////////////

    // Prototypes for serialization/deserialization functions
    template<typename Container>
    inline uint64_t SerializedSizeEigenType(const Container& value);

    // For fixed-size containers only (others have a uint64_t size header first)
    template<typename Container>
    inline uint64_t SerializedSizeEigenType(void);

    template<typename Container>
    inline uint64_t SerializeEigenType(const Container& value, std::vector<uint8_t>& buffer);

    template<typename Container>
    inline std::pair<Container, uint64_t> DeserializeEigenType(const std::vector<uint8_t>& buffer, const uint64_t current);

    // Concrete implementations
    template<>
    inline uint64_t SerializedSizeEigenType(const Eigen::VectorXd& value)
    {
        (void)(value);
        return (uint64_t)((1 * sizeof(uint64_t)) + ((size_t)value.size() * sizeof(double))); // Space for a uint64_t size header and the data
    }

    template<>
    inline uint64_t SerializeEigenType(const Eigen::VectorXd& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        const uint64_t serialized_size = SerializedSizeEigenType(value);
        std::vector<uint8_t> temp_buffer(serialized_size, 0x00);
        // Make the header
        const uint64_t size_header = (uint64_t)value.size();
        memcpy(&temp_buffer.front(), & size_header, sizeof(size_header));
        // Copy the data
        memcpy(&(temp_buffer[sizeof(size_header)]), value.data(), (serialized_size - sizeof(size_header)));
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return serialized_size;
    }

    template<>
    inline std::pair<Eigen::VectorXd, uint64_t> DeserializeEigenType<Eigen::VectorXd>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + sizeof(uint64_t)) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        // Load the header
        uint64_t size_header = 0u;
        memcpy(&size_header, &buffer[current], sizeof(uint64_t));
        // Check buffer size
        Eigen::VectorXd temp_value = Eigen::VectorXd::Zero((ssize_t)size_header);
        const uint64_t serialized_size = SerializedSizeEigenType(temp_value);
        assert((current + serialized_size) <= buffer.size());
        // Load from the buffer
        memcpy(temp_value.data(), &buffer[current + sizeof(size_header)], (serialized_size - sizeof(size_header)));
        return std::make_pair(temp_value, serialized_size);
    }

    template<>
    inline uint64_t SerializedSizeEigenType(const Eigen::Vector3d& value)
    {
        (void)(value);
        return (uint64_t)(3 * sizeof(double));
    }

    template<>
    inline uint64_t SerializedSizeEigenType<Eigen::Vector3d>(void)
    {
        return (uint64_t)(3 * sizeof(double));
    }

    template<>
    inline uint64_t SerializeEigenType(const Eigen::Vector3d& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        std::vector<uint8_t> temp_buffer(SerializedSizeEigenType<Eigen::Vector3d>(), 0x00);
        memcpy(&temp_buffer.front(), value.data(), SerializedSizeEigenType<Eigen::Vector3d>());
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return SerializedSizeEigenType<Eigen::Vector3d>();
    }

    template<>
    inline std::pair<Eigen::Vector3d, uint64_t> DeserializeEigenType<Eigen::Vector3d>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + SerializedSizeEigenType<Eigen::Vector3d>()) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        Eigen::Vector3d temp_value;
        memcpy(temp_value.data(), &buffer[current], SerializedSizeEigenType<Eigen::Vector3d>());
        return std::make_pair(temp_value, SerializedSizeEigenType<Eigen::Vector3d>());
    }

    template<>
    inline uint64_t SerializedSizeEigenType(const Eigen::Matrix<double, 6, 1>& value)
    {
        (void)(value);
        return (uint64_t)(6 * sizeof(double));
    }

    template<>
    inline uint64_t SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>(void)
    {
        return (uint64_t)(6 * sizeof(double));
    }

    template<>
    inline uint64_t SerializeEigenType(const Eigen::Matrix<double, 6, 1>& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        std::vector<uint8_t> temp_buffer(SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>(), 0x00);
        memcpy(&temp_buffer.front(), value.data(), SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>());
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>();
    }

    template<>
    inline std::pair<Eigen::Matrix<double, 6, 1>, uint64_t> DeserializeEigenType<Eigen::Matrix<double, 6, 1>>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>()) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        Eigen::Matrix<double, 6, 1> temp_value;
        memcpy(temp_value.data(), &buffer[current], SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>());
        return std::make_pair(temp_value, SerializedSizeEigenType<Eigen::Matrix<double, 6, 1>>());
    }

    template<>
    inline uint64_t SerializedSizeEigenType(const Eigen::Isometry3d& value)
    {
        (void)(value);
        return (uint64_t)(16 * sizeof(double));
    }

    template<>
    inline uint64_t SerializedSizeEigenType<Eigen::Isometry3d>(void)
    {
        return (uint64_t)(16 * sizeof(double));
    }

    template<>
    inline uint64_t SerializeEigenType(const Eigen::Isometry3d& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        std::vector<uint8_t> temp_buffer(SerializedSizeEigenType<Eigen::Isometry3d>(), 0x00);
        memcpy(&temp_buffer.front(), value.matrix().data(), SerializedSizeEigenType<Eigen::Isometry3d>());
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return SerializedSizeEigenType<Eigen::Isometry3d>();
    }

    template<>
    inline std::pair<Eigen::Isometry3d, uint64_t> DeserializeEigenType<Eigen::Isometry3d>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + SerializedSizeEigenType<Eigen::Isometry3d>()) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        Eigen::Isometry3d temp_value;
        memcpy(temp_value.matrix().data(), &buffer[current], SerializedSizeEigenType<Eigen::Isometry3d>());
        return std::make_pair(temp_value, SerializedSizeEigenType<Eigen::Isometry3d>());
    }
}

#endif // SERIALIZATION_EIGEN_HPP
