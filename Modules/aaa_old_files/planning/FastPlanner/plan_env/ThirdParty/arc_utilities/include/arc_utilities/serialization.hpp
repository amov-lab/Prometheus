#ifndef SERIALIZATION_HPP
#define SERIALIZATION_HPP

#include <cstring>
#include <vector>
#include <map>
#include <functional>
#include <assert.h>

namespace arc_utilities
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////                                       PROTOTYPES ONLY                                         /////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename T>
    inline uint64_t SerializeFixedSizePOD(
            const T& item_to_serialize,
            std::vector<uint8_t>& buffer);

    template<typename T>
    inline std::pair<T, uint64_t> DeserializeFixedSizePOD(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    template<typename T, typename Allocator = std::allocator<T>>
    inline uint64_t SerializeVector(
            const std::vector<T, Allocator>& vec_to_serialize, std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& item_serializer);

    template<typename T, typename Allocator = std::allocator<T>>
    inline std::pair<std::vector<T, Allocator>, uint64_t> DeserializeVector(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& item_deserializer);

    template<typename Key, typename T, typename Compare = std::less<Key>, typename Allocator = std::allocator<std::pair<const Key, T>>>
    inline uint64_t SerializeMap(
            const std::map<Key, T, Compare, Allocator>& map_to_serialize,
            std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const Key&, std::vector<uint8_t>&)>& key_serializer,
            const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer);

    template<typename Key, typename T, typename Compare = std::less<Key>, typename Allocator = std::allocator<std::pair<const Key, T>>>
    inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t> DeserializeMap(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<Key, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
            const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer);

    template<typename First, typename Second>
    inline uint64_t SerializePair(
            const std::pair<First, Second>& pair_to_serialize,
            std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const First&, std::vector<uint8_t>&)>& first_serializer,
            const std::function<uint64_t(const Second&, std::vector<uint8_t>&)>& second_serializer);

    template<typename First, typename Second>
    inline const std::pair<std::pair<First, Second>, uint64_t> DeserializePair(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<First, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& first_deserializer,
            const std::function<std::pair<Second, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& second_deserializer);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////                                   IMPLEMENTATIONS ONLY                                        /////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename T>
    inline uint64_t SerializeFixedSizePOD(
            const T& item_to_serialize,
            std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // Fixed-size serialization via memcpy
        std::vector<uint8_t> temp_buffer(sizeof(item_to_serialize), 0x00);
        std::memcpy(&temp_buffer[0], &item_to_serialize, sizeof(item_to_serialize));
        // Move to buffer
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename T>
    inline std::pair<T, uint64_t> DeserializeFixedSizePOD(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        T temp_item;
        assert(current <= buffer.size());
        assert((current + sizeof(temp_item)) <= buffer.size());
        std::memcpy(&temp_item, &buffer[current], sizeof(temp_item));
        return std::make_pair(temp_item, sizeof(temp_item));
    }

    template<typename CharType>
    inline uint64_t SerializeString(
            const std::basic_string<CharType>& str_to_serialize,
            std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)str_to_serialize.size();
        SerializeFixedSizePOD<uint64_t>(size, buffer);
        // Serialize the contained items
        for (size_t idx = 0; idx < size; idx++)
        {
            const CharType& current = str_to_serialize[idx];
            SerializeFixedSizePOD<CharType>(current, buffer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename CharType>
    inline std::pair<std::basic_string<CharType>, uint64_t> DeserializeString(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        // First, try to load the header
        assert(current < buffer.size());
        uint64_t current_position = current;
        // Load the header
        const std::pair<uint64_t, uint64_t> deserialized_size = DeserializeFixedSizePOD<uint64_t>(buffer, current_position);
        const uint64_t size = deserialized_size.first;
        current_position += deserialized_size.second;
        // Deserialize the items
        std::basic_string<CharType> deserialized;
        deserialized.reserve(size);
        for (uint64_t idx = 0; idx < size; idx++)
        {
            const std::pair<CharType, uint64_t> deserialized_char = DeserializeFixedSizePOD<CharType>(buffer, current_position);
            deserialized.push_back(deserialized_char.first);
            current_position += deserialized_char.second;
        }
        deserialized.shrink_to_fit();
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    template<typename T, typename Allocator>
    inline uint64_t SerializeVector(
            const std::vector<T, Allocator>& vec_to_serialize,
            std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& item_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)vec_to_serialize.size();
        SerializeFixedSizePOD<uint64_t>(size, buffer);
        // Serialize the contained items
        for (size_t idx = 0; idx < size; idx++)
        {
            const T& current = vec_to_serialize[idx];
            item_serializer(current, buffer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename T, typename Allocator>
    inline std::pair<std::vector<T, Allocator>, uint64_t> DeserializeVector(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& item_deserializer)
    {
        // First, try to load the header
        assert(current < buffer.size());
        uint64_t current_position = current;
        // Load the header
        const std::pair<uint64_t, uint64_t> deserialized_size = DeserializeFixedSizePOD<uint64_t>(buffer, current_position);
        const uint64_t size = deserialized_size.first;
        current_position += deserialized_size.second;
        // Deserialize the items
        std::vector<T, Allocator> deserialized;
        deserialized.reserve(size);
        for (uint64_t idx = 0; idx < size; idx++)
        {
            const std::pair<T, uint64_t> deserialized_item = item_deserializer(buffer, current_position);
            deserialized.push_back(deserialized_item.first);
            current_position += deserialized_item.second;
        }
        deserialized.shrink_to_fit();
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    template<typename Key, typename T, typename Compare, typename Allocator>
    inline uint64_t SerializeMap(
            const std::map<Key, T, Compare, Allocator>& map_to_serialize,
            std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const Key&, std::vector<uint8_t>&)>& key_serializer,
            const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)map_to_serialize.size();
        SerializeFixedSizePOD<uint64_t>(size, buffer);
        // Serialize the contained items
        typename std::map<Key, T, Compare, Allocator>::const_iterator itr;
        for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
        {
            SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename Key, typename T, typename Compare, typename Allocator>
    inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t> DeserializeMap(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<Key, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& key_deserializer,
            const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
    {
        // First, try to load the header
        assert(current < buffer.size());
        uint64_t current_position = current;
        // Load the header
        const std::pair<uint64_t, uint64_t> deserialized_size = DeserializeFixedSizePOD<uint64_t>(buffer, current_position);
        const uint64_t size = deserialized_size.first;
        current_position += deserialized_size.second;
        // Deserialize the items
        std::map<Key, T, Compare, Allocator> deserialized;
        for (uint64_t idx = 0; idx < size; idx++)
        {
            std::pair<std::pair<Key, T>, uint64_t> deserialized_pair = DeserializePair(buffer, current_position, key_deserializer, value_deserializer);
            deserialized.insert(deserialized_pair.first);
            current_position += deserialized_pair.second;
        }
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    template<typename First, typename Second>
    inline uint64_t SerializePair(
            const std::pair<First, Second>& pair_to_serialize,
            std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const First&, std::vector<uint8_t>&)>& first_serializer,
            const std::function<uint64_t(const Second&, std::vector<uint8_t>&)>& second_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        uint64_t running_total = 0u;
        // Write each element of the pair into the buffer
        running_total += first_serializer(pair_to_serialize.first, buffer);
        running_total += second_serializer(pair_to_serialize.second, buffer);
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        assert(bytes_written == running_total);
        return bytes_written;
    }

    template<typename First, typename Second>
    inline const std::pair<std::pair<First, Second>, uint64_t> DeserializePair(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<First, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& first_deserializer,
            const std::function<std::pair<Second, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& second_deserializer)
    {
        assert(current < buffer.size());
        // Deserialize each item in the pair individually
        uint64_t current_position = current;
        const std::pair<First, uint64_t> deserialized_first = first_deserializer(buffer, current_position);
        current_position += deserialized_first.second;
        const std::pair<Second, uint64_t> deserialized_second = second_deserializer(buffer, current_position);
        current_position += deserialized_second.second;
        // Build the resulting pair
        // TODO: Why can't I used make_pair here?
        const std::pair<First, Second> deserialized(deserialized_first.first, deserialized_second.first);
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }
}

#endif // SERIALIZATION_HPP
