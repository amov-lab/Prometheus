#include <stdlib.h>
#include <functional>
#include <omp.h>
#ifdef ENABLE_PARALLEL_ROADMAP
    #ifndef ENABLE_PARALLEL_K_NEAREST_NEIGHBORS
        #define ENABLE_PARALLEL_K_NEAREST_NEIGHBORS
        #include <arc_utilities/arc_helpers.hpp>
        #undef ENABLE_PARALLEL_K_NEAREST_NEIGHBORS
    #else
        #include <arc_utilities/arc_helpers.hpp>
    #endif
#else
    #include <arc_utilities/arc_helpers.hpp>
#endif
#include <arc_utilities/dijkstras.hpp>


#ifndef SIMPLE_PRM_PLANNER_HPP
#define SIMPLE_PRM_PLANNER_HPP

namespace simple_prm_planner
{
    class SimpleGeometricPrmPlanner
    {
    protected:

        SimpleGeometricPrmPlanner() {}

        enum NNDistanceDirection {ROADMAP_TO_NEW_STATE, NEW_STATE_TO_ROADMAP};

        template<typename T, typename Allocator = std::allocator<T>>
        static int64_t AddNodeToRoadmap(
                const T& state,
                const NNDistanceDirection nn_distance_direction,
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<double(const T&, const T&)>& distance_fn,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            // Make the node->graph or graph->node distance function as needed, then call KNN
            std::function<double(const arc_dijkstras::GraphNode<T, Allocator>&, const T&)> graph_distance_fn = nullptr;
            if (nn_distance_direction == ROADMAP_TO_NEW_STATE)
            {
                graph_distance_fn = [&] (const arc_dijkstras::GraphNode<T, Allocator>& node, const T& state)
                {
                    return distance_fn(node.GetValueImmutable(), state);
                };
            }
            else
            {
                graph_distance_fn = [&] (const arc_dijkstras::GraphNode<T, Allocator>& node, const T& state)
                {
                    return distance_fn(state, node.GetValueImmutable());
                };
            }
            const std::vector<std::pair<int64_t, double>> nearest_neighbors =
                    arc_helpers::GetKNearestNeighbors(roadmap.GetNodesImmutable(), state, graph_distance_fn, K);

            // Check to see this node is already in the PRM, if it is, don't re-add it, returning the existing state index
            for (const auto& nn_result : nearest_neighbors)
            {
                // If the distance is zero, check that the states are also equal
                // This allows for a psuedo-metric and not just a true metric to be used as the distance function
                if (nn_result.second == 0)
                {
                    const int64_t neighbour_node_idx = nn_result.first;
                    const T& neighbour_state = roadmap.GetNodeImmutable(neighbour_node_idx).GetValueImmutable();
                    if (neighbour_state == state)
                    {
                        return neighbour_node_idx;
                    }
                }
            }

            // Add the new node AFTER KNN is performed
            const int64_t new_node_index = roadmap.AddNode(state);
            // Parallelize the collision-checking and distance computation
            std::vector<std::pair<double, double>> nearest_neighbors_distances(nearest_neighbors.size());
#ifdef ENABLE_PARALLEL_ROADMAP
            #pragma omp parallel for
#endif
            for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
            {
                const std::pair<int64_t, double>& nearest_neighbor = nearest_neighbors[idx];
                const int64_t nearest_neighbor_index = nearest_neighbor.first;
                const double nearest_neighbor_distance = nearest_neighbor.second;
                const T& nearest_neighbor_state = roadmap.GetNodeImmutable(nearest_neighbor_index).GetValueImmutable();
                if (edge_validity_check_fn(nearest_neighbor_state, state))
                {
                    if (distance_is_symmetric)
                    {
                        nearest_neighbors_distances[idx] = std::make_pair(nearest_neighbor_distance, nearest_neighbor_distance);
                    }
                    else
                    {
                        const double reverse_graph_distance = distance_fn(state, roadmap.GetNodeImmutable(nearest_neighbor_index).GetValueImmutable());
                        if (nn_distance_direction == ROADMAP_TO_NEW_STATE)
                        {
                            nearest_neighbors_distances[idx] = std::make_pair(nearest_neighbor_distance, reverse_graph_distance);
                        }
                        else if (nn_distance_direction == ROADMAP_TO_NEW_STATE)
                        {
                            nearest_neighbors_distances[idx] = std::make_pair(reverse_graph_distance, nearest_neighbor_distance);
                        }
                        else
                        {
                            assert(false && "This code should not be reachable");
                        }
                    }
                }
                else
                {
                    nearest_neighbors_distances[idx] = std::make_pair(-1.0, -1.0);
                }
            }
            // THIS MUST BE SERIAL - add edges to roadmap
            for (size_t idx = 0; idx < nearest_neighbors.size(); idx++)
            {
                const std::pair<int64_t, double>& nearest_neighbor = nearest_neighbors[idx];
                const int64_t nearest_neighbor_index = nearest_neighbor.first;
                const std::pair<double, double>& nearest_neighbor_distances = nearest_neighbors_distances[idx];
                if (nearest_neighbor_distances.first >= 0.0 && nearest_neighbor_distances.second >= 0.0)
                {
                    // Add the edges individually to allow for different distances in each direction
                    roadmap.AddEdgeBetweenNodes(nearest_neighbor_index, new_node_index, nearest_neighbor_distances.first);
                    roadmap.AddEdgeBetweenNodes(new_node_index, nearest_neighbor_index, nearest_neighbor_distances.second);
                }
            }
            return new_node_index;
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static std::vector<T, Allocator> ExtractSolutionPath(
                const arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::vector<int64_t>& solution_path_indices)
        {
            std::vector<T, Allocator> solution_path;
            solution_path.reserve(solution_path_indices.size());
            for (size_t idx = 0; idx < solution_path_indices.size(); idx++)
            {
                const int64_t path_index = solution_path_indices[idx];
                solution_path.push_back(roadmap.GetNodeImmutable(path_index).GetValueImmutable());
            }
            solution_path.shrink_to_fit();
            return solution_path;
        }

    public:

        template<typename T, typename Allocator = std::allocator<T>>
        static void ExtendRoadMap(
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<T(void)>& sampling_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const std::function<bool(const T&)>& state_validity_check_fn,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<bool(void)>& termination_check_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            while (!termination_check_fn())
            {
                const T random_state = sampling_fn();
                if (state_validity_check_fn(random_state))
                {
                    AddNodeToRoadmap(random_state, ROADMAP_TO_NEW_STATE, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
                }
            }
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static arc_dijkstras::Graph<T, Allocator> BuildRoadMap(
                const std::function<T(void)>& sampling_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const std::function<bool(const T&)>& state_validity_check_fn,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<bool(void)>& termination_check_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            arc_dijkstras::Graph<T, Allocator> roadmap;
            ExtendRoadMap(roadmap, sampling_fn, distance_fn, state_validity_check_fn, edge_validity_check_fn, termination_check_fn, K, distance_is_symmetric);
            return roadmap;
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static void UpdateRoadMapEdges(
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn)
        {
            assert(roadmap.CheckGraphLinkage());
#ifdef ENABLE_PARALLEL_ROADMAP
            #pragma omp parallel for
#endif
            for (size_t current_node_index = 0; current_node_index < roadmap.GetNodesImmutable().size(); current_node_index++)
            {
                arc_dijkstras::GraphNode<T, Allocator>& current_node = roadmap.GetNodeMutable(current_node_index);
                std::vector<arc_dijkstras::GraphEdge>& current_node_out_edges = current_node.GetOutEdgesMutable();
                for (size_t out_edge_idx = 0; out_edge_idx < current_node_out_edges.size(); out_edge_idx++)
                {
                    arc_dijkstras::GraphEdge& current_out_edge = current_node_out_edges[out_edge_idx];
                    const int64_t other_node_idx = current_out_edge.GetToIndex();
                    arc_dijkstras::GraphNode<T, Allocator>& other_node = roadmap.GetNodeMutable(other_node_idx);
                    std::vector<arc_dijkstras::GraphEdge>& other_node_in_edges = other_node.GetInEdgesMutable();
                    // If the edge is not valid, set the weight to infinity, otherwise use the distance function
                    double updated_weight = std::numeric_limits<double>::infinity();
                    if (edge_validity_check_fn(current_node.GetValueImmutable(), other_node.GetValueImmutable()))
                    {
                        updated_weight = distance_fn(current_node.GetValueImmutable(), other_node.GetValueImmutable());
                    }
                    // Update our out edge
                    current_out_edge.SetWeight(updated_weight);
                    // Update the other node's in edges
                    for (size_t in_edge_idx = 0; in_edge_idx < other_node_in_edges.size(); in_edge_idx++)
                    {
                        arc_dijkstras::GraphEdge& other_in_edge = other_node_in_edges[in_edge_idx];
                        if (other_in_edge.GetFromIndex() == current_node_index)
                        {
                            other_in_edge.SetWeight(updated_weight);
                        }
                    }
                }
            }
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> QueryPathAndAddNodesMultiStartSingleGoal(
                const std::vector<T, Allocator>& starts,
                const T& goal,
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            // Add the multiple start nodes to the roadmap
            std::vector<int64_t> start_node_indices(starts.size());
            for (size_t start_idx = 0; start_idx < starts.size(); start_idx++)
            {
                const T& start = starts[start_idx];
                start_node_indices[start_idx] = AddNodeToRoadmap(start, NEW_STATE_TO_ROADMAP, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            }
            // Add the goal node to the roadmap
            const int64_t goal_node_index = AddNodeToRoadmap(goal, ROADMAP_TO_NEW_STATE, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Call Dijkstra's
            const auto dijkstras_solution = arc_dijkstras::SimpleDijkstrasAlgorithm<T, Allocator>::PerformDijkstrasAlgorithm(roadmap, goal_node_index);
            // Identify the lowest-distance starting state
            const std::pair<std::vector<int64_t>, std::vector<double>>& solution_map_distances = dijkstras_solution.second;
            double best_start_node_distance = std::numeric_limits<double>::infinity();
            int64_t best_start_node_index = -1;
            for (size_t start_idx = 0; start_idx < starts.size(); start_idx++)
            {
                const int64_t start_node_index = start_node_indices[start_idx];
                const double start_node_distance = solution_map_distances.second[start_node_index];
                if (start_node_distance < best_start_node_distance)
                {
                    best_start_node_distance = start_node_distance;
                    best_start_node_index = start_node_index;
                }
            }
            const int64_t start_node_index = best_start_node_index;
            const double start_node_distance = best_start_node_distance;
            // Extract solution path
            if (std::isinf(start_node_distance))
            {
                return std::make_pair(std::vector<T, Allocator>(), std::numeric_limits<double>::infinity());
            }
            else
            {
                std::vector<int64_t> solution_path_indices;
                solution_path_indices.push_back(start_node_index);
                int64_t previous_index = solution_map_distances.first[start_node_index];
                while (previous_index >= 0)
                {
                    const int64_t current_index = previous_index;
                    solution_path_indices.push_back(current_index);
                    if (current_index == goal_node_index)
                    {
                        break;
                    }
                    else
                    {
                        previous_index = solution_map_distances.first[current_index];
                    }
                }
                const std::vector<T, Allocator> solution_path = ExtractSolutionPath(roadmap, solution_path_indices);
                return std::make_pair(solution_path, start_node_distance);
            }
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> QueryPathAndAddNodesSingleStartSingleGoal(
                const T& start,
                const T& goal,
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true,
                const bool limit_astar_pqueue_duplicates = true)
        {
            // Add the start node to the roadmap
            const int64_t start_node_index = AddNodeToRoadmap(start, NEW_STATE_TO_ROADMAP, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Add the goal node to the roadmap
            const int64_t goal_node_index = AddNodeToRoadmap(goal, ROADMAP_TO_NEW_STATE, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Call graph A*
            const std::pair<std::vector<int64_t>, double> astar_result = arc_dijkstras::SimpleGraphAstar<T, Allocator>::PerformAstar(
                        roadmap, start_node_index, goal_node_index, distance_fn, limit_astar_pqueue_duplicates);
            // Convert the solution path from A* provided as indices into real states
            const std::vector<int64_t>& solution_path_indices = astar_result.first;
            std::vector<T, Allocator> solution_path;
            solution_path.reserve(astar_result.first.size());
            for (size_t idx = 0; idx < solution_path_indices.size(); idx++)
            {
                const int64_t path_index = solution_path_indices[idx];
                solution_path.push_back(roadmap.GetNodeImmutable(path_index).GetValueImmutable());
            }
            solution_path.shrink_to_fit();
            return std::make_pair(solution_path, astar_result.second);
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> LazyQueryPathAndAddNodesSingleStartSingleGoal(
                const T& start,
                const T& goal,
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K, const bool distance_is_symmetric=true,
                const bool limit_astar_pqueue_duplicates = true)
        {
            // Add the start node to the roadmap
            const int64_t start_node_index = AddNodeToRoadmap(start, NEW_STATE_TO_ROADMAP, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Add the goal node to the roadmap
            const int64_t goal_node_index = AddNodeToRoadmap(goal, ROADMAP_TO_NEW_STATE, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Call graph A*
            const std::pair<std::vector<int64_t>, double> astar_result = arc_dijkstras::SimpleGraphAstar<T, Allocator>::PerformLazyAstar(
                        roadmap, start_node_index, goal_node_index, edge_validity_check_fn, distance_fn, distance_fn, limit_astar_pqueue_duplicates);
            // Convert the solution path from A* provided as indices into real states
            const std::vector<T, Allocator> solution_path = ExtractSolutionPath(roadmap, astar_result.first);
            return std::make_pair(solution_path, astar_result.second);
        }

        template<typename T, typename Allocator = std::allocator<T>, typename Generator = std::mt19937_64>
        static std::pair<std::vector<T, Allocator>, double> QueryPathAndAddNodesSingleStartSingleGoalRandomWalk(
                const T& start,
                const T& goal,
                Generator& generator,
                arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            // Add the start node to the roadmap
            const int64_t start_node_index = AddNodeToRoadmap(start, NEW_STATE_TO_ROADMAP, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Add the goal node to the roadmap
            const int64_t goal_node_index = AddNodeToRoadmap(goal, ROADMAP_TO_NEW_STATE, roadmap, distance_fn, edge_validity_check_fn, K, distance_is_symmetric);
            // Call the random walk algorithm
            const auto random_walk_result = arc_dijkstras::GraphRandomWalk<T, Allocator>::PerformRandomWalk(roadmap, start_node_index, goal_node_index, generator);
            // Convert the result into a path and return it
            const auto solution_path = ExtractSolutionPath(roadmap, random_walk_result);
            const auto distance = EigenHelpers::CalculateTotalDistance(solution_path, distance_fn);
            return std::make_pair(solution_path, distance);
        }

        // TODO - figure out a better way to balance parallelism between KNN queries inside path calls and multiple calls to Dijkstras
        template<typename T, typename Allocator = std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> QueryPathMultiStartMultiGoal(
                const std::vector<T, Allocator>& starts,
                const std::vector<T, Allocator>& goals,
                const arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            std::vector<std::pair<std::vector<T, Allocator>, double>> possible_solutions(goals.size());
            for (size_t goal_idx = 0; goal_idx < goals.size(); goal_idx++)
            {
                possible_solutions[goal_idx] = QueryPathMultiStartSingleGoal(starts, goals[goal_idx], roadmap, edge_validity_check_fn, distance_fn, K, distance_is_symmetric);
            }
            const double best_solution_distance = std::numeric_limits<double>::infinity();
            const int64_t best_solution_index = -1;
            for (size_t goal_idx = 0; goal_idx < goals.size(); goal_idx++)
            {
                const double solution_distance = possible_solutions[goal_idx].second;
                if (solution_distance < best_solution_distance)
                {
                    best_solution_distance = solution_distance;
                    best_solution_index = goal_idx;
                }
            }
            if ((best_solution_index >= 0) && (best_solution_distance < std::numeric_limits<double>::infinity()))
            {
                return possible_solutions[best_solution_index];
            }
            else
            {
                return std::make_pair(std::vector<T, Allocator>(), std::numeric_limits<double>::infinity());
            }
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> QueryPathMultiStartSingleGoal(
                const std::vector<T, Allocator>& starts,
                const T& goal,
                const arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            arc_dijkstras::Graph<T, Allocator> working_copy = roadmap;
            return QueryPathAndAddNodesMultiStartSingleGoal(starts, goal, working_copy, edge_validity_check_fn, distance_fn, K, distance_is_symmetric);
        }

        template<typename T, typename Allocator = std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> QueryPathSingleStartSingleGoal(
                const T& start,
                const T& goal,
                const arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            arc_dijkstras::Graph<T, Allocator> working_copy = roadmap;
            return QueryPathAndAddNodesSingleStartSingleGoal(start, goal, working_copy, edge_validity_check_fn, distance_fn, K, distance_is_symmetric);
        }

        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, double> LazyQueryPathSingleStartSingleGoal(
                const T& start, const T& goal, const arc_dijkstras::Graph<T, Allocator>& roadmap,
                const std::function<bool(const T&, const T&)>& edge_validity_check_fn,
                const std::function<double(const T&, const T&)>& distance_fn,
                const size_t K,
                const bool distance_is_symmetric = true)
        {
            arc_dijkstras::Graph<T, Allocator> working_copy = roadmap;
            return LazyQueryPathAndAddNodesSingleStartSingleGoal(start, goal, working_copy, edge_validity_check_fn, distance_fn, K, distance_is_symmetric);
        }

        // TODO update to provide lazy and non-lazy variants of single start/single goal
    };
}

#endif // SIMPLE_PRM_PLANNER
