#ifndef GET_NEIGHBOURS_HPP
#define GET_NEIGHBOURS_HPP

#include <algorithm>
#include <functional>
#include <vector>

namespace arc_utilities
{
    namespace GetNeighbours
    {
        template<typename ConfigType, typename StepType, typename Allocator = std::allocator<ConfigType>>
        inline std::vector<ConfigType, Allocator> TwoDimensional8Connected(
                const ConfigType& config,
                const StepType& min_x,
                const StepType& max_x,
                const StepType& min_y,
                const StepType& max_y,
                const StepType& step_size,
                const std::function<ConfigType(const ConfigType&)>& round_to_grid_fn,
                const std::function<bool(const ConfigType&)>& validity_check_fn)
        {
            std::vector<ConfigType, Allocator> neighbours;
            neighbours.reserve(8);

            const StepType x_min = std::max(min_x, config[0] - step_size);
            const StepType x_max = std::min(max_x, config[0] + step_size);

            const StepType y_min = std::max(min_y, config[1] - step_size);
            const StepType y_max = std::min(max_y, config[1] + step_size);

            const ConfigType min_vector = round_to_grid_fn(ConfigType(x_min, y_min));
            const ConfigType max_vector = round_to_grid_fn(ConfigType(x_max, y_max));

            for (int x_offset = -1; x_offset <= 1; ++x_offset)
            {
                const double x = config[0] + step_size * x_offset;
                for (int y_offset = -1; y_offset <= 1; ++y_offset)
                {
                    const double y = config[1] + step_size * y_offset;
                    const ConfigType neighbour = round_to_grid_fn(ConfigType(x, y));

                    if (min_vector[0] <= neighbour[0] && neighbour[0] <= max_vector[0] &&
                        min_vector[1] <= neighbour[1] && neighbour[1] <= max_vector[1] &&
                        (neighbour[0] != config[0] || neighbour[1] != config[1]) &&
                        validity_check_fn(neighbour) == true)
                    {
                        neighbours.push_back(neighbour);
                    }
                }
            }

            neighbours.shrink_to_fit();
            return neighbours;
        }

        template<typename ConfigType, typename StepType, typename Allocator = std::allocator<ConfigType>>
        inline std::vector<ConfigType, Allocator> ThreeDimensional8Connected(
                const ConfigType& config,
                const StepType& min_x,
                const StepType& max_x,
                const StepType& min_y,
                const StepType& max_y,
                const StepType& min_z,
                const StepType& max_z,
                const StepType& step_size,
                const std::function<ConfigType(const ConfigType&)>& round_to_grid_fn,
                const std::function<bool(const ConfigType&)>& validity_check_fn)
        {
            std::vector<ConfigType, Allocator> neighbours;
            neighbours.reserve(26);

            const StepType x_min = std::max(min_x, config[0] - step_size);
            const StepType x_max = std::min(max_x, config[0] + step_size);

            const StepType y_min = std::max(min_y, config[1] - step_size);
            const StepType y_max = std::min(max_y, config[1] + step_size);

            const StepType z_min = std::max(min_z, config[2] - step_size);
            const StepType z_max = std::min(max_z, config[2] + step_size);

            const ConfigType min_vector = round_to_grid_fn(ConfigType(x_min, y_min, z_min));
            const ConfigType max_vector = round_to_grid_fn(ConfigType(x_max, y_max, z_max));

            for (int x_offset = -1; x_offset <= 1; ++x_offset)
            {
                const double x = config[0] + step_size * x_offset;
                for (int y_offset = -1; y_offset <= 1; ++y_offset)
                {
                    const double y = config[1] + step_size * y_offset;
                    for (int z_offset = -1; z_offset <= 1; ++z_offset)
                    {
                        const double z = config[2] + step_size * z_offset;
                        const ConfigType neighbour = round_to_grid_fn(ConfigType(x, y, z));

                        if (min_vector[0] <= neighbour[0] && neighbour[0] <= max_vector[0] &&
                            min_vector[1] <= neighbour[1] && neighbour[1] <= max_vector[1] &&
                            min_vector[2] <= neighbour[2] && neighbour[2] <= max_vector[2] &&
                            (neighbour[0] != config[0] || neighbour[1] != config[1] || neighbour[2] != config[2]) &&
                            validity_check_fn(neighbour) == true)
                        {
                            neighbours.push_back(neighbour);
                        }
                    }
                }
            }

            neighbours.shrink_to_fit();
            return neighbours;
        }
    }
}

#endif // GET_NEIGHBOURS_HPP
