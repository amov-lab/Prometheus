#include "arc_utilities/first_order_deformation.h"

#include <queue>
#include <Eigen/Core>

using namespace arc_utilities;
using namespace FirstOrderDeformation;

static double ConfigTypeDistance(const ConfigType& c1, const ConfigType& c2)
{
    return Eigen::Vector2d((double)(c1.first - c2.first), (double)(c1.second - c2.second)).norm();
}

static std::vector<ConfigType> GetNeighbours(const ConfigType& config, const ssize_t rows, const ssize_t cols, const ValidityCheckFnType& validity_check_fn)
{
    std::vector<ConfigType> neighbours;
    neighbours.reserve(8);

    const ssize_t row_min = std::max(0L, config.first - 1);
    const ssize_t row_max = std::min(rows - 1, config.first + 1);

    const ssize_t col_min = std::max(0L, config.second - 1);
    const ssize_t col_max = std::min(cols - 1, config.second + 1);

    for (ssize_t col = col_min; col <= col_max; col++)
    {
        for (ssize_t row = row_min; row <= row_max; row++)
        {
            if (!(row == config. first && col == config.second) && validity_check_fn(row, col) == true)
            {
                neighbours.push_back(ConfigType(row, col));
            }
        }
    }

    return neighbours;
}

bool FirstOrderDeformation::CheckFirstOrderDeformation(const ssize_t rows, const ssize_t cols, const ValidityCheckFnType& validity_check_fn)
{
    struct BestFirstSearchComparator
    {
        public:
            // Defines a "less" operation"; by using "greater" the smallest element will appear at the top of the priority queue
            bool operator()(const ConfigAndDistType& c1, const ConfigAndDistType& c2) const
            {
                // We want to explore the one with the smaller expected distance
                return (c1.second > c2.second);
            }
    };

    assert(rows > 0 && cols > 0);
    typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> ArrayXb;

    // Setup the strucutres needed to keep track of the search
    std::priority_queue<ConfigAndDistType, std::vector<ConfigAndDistType>, BestFirstSearchComparator> frontier;
    ArrayXb explored = ArrayXb::Constant(rows, cols, false);

    // Setup the start and goal
    const ConfigType start(0, 0), goal(rows - 1, cols - 1);
    const double start_heuristic_distance = ConfigTypeDistance(start, goal);
    frontier.push(ConfigAndDistType(start, start_heuristic_distance));

    // Perform the best first search
    bool path_found = false;
    while (!path_found && frontier.size() > 0)
    {
        const ConfigAndDistType current = frontier.top();
        frontier.pop();
        const ConfigType& current_config = current.first;

        if (current_config.first == goal.first && current_config.second == goal.second)
        {
            path_found = true;
        }
        // Double check if we've already explored this node:
        //    a single node can be inserted into the frontier multiple times at the same or different priorities
        //    so we want to avoid the expense of re-exploring it, and just discard this one once we pop it
        else if (explored(current_config.first, current_config.second) == false)
        {
            explored(current_config.first, current_config.second) = true;

            // Expand the node to find all neighbours, adding them to the frontier if we have not already explored them
            const auto neighbours = GetNeighbours(current_config, rows, cols, validity_check_fn);
            for (const auto neighbour : neighbours)
            {
                // Check if we've already explored this neighbour to avoid re-adding it to the frontier
                if (explored(neighbour.first, neighbour.second) == false)
                {
                    // We only need the heuristic distance as we are doing a best first search
                    const double neighbour_heuristic_distance = ConfigTypeDistance(neighbour, goal);
                    frontier.push(ConfigAndDistType(neighbour, neighbour_heuristic_distance));
                }
            }
        }
    }

    return path_found;
}
