#ifndef SIMPLE_ASTAR_PLANNER_HPP
#define SIMPLE_ASTAR_PLANNER_HPP

#include <vector>
#include <map>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <chrono>

#include <arc_utilities/eigen_helpers.hpp>

//#include <ros/ros.h>
//#include <visualization_msgs/Marker.h>
//#include <arc_utilities/arc_helpers.hpp>
//#include <arc_utilities/ros_helpers.hpp>
//#include <arc_utilities/eigen_helpers_conversions.hpp>

namespace simple_astar_planner
{
    template<typename ConfigType, typename Allocator = std::allocator<ConfigType>>
    class SimpleAStarPlanner
    {
        protected:
            SimpleAStarPlanner() {}

            typedef std::pair<ConfigType, double> ConfigAndDistType;
            struct AStarComparator
            {
                public:
                    AStarComparator(const std::function<double(const ConfigType&)>& heuristic_fn)
                        : heuristic_fn_(heuristic_fn)
                    {}

                    // Defines a "less" operation"; by using "greater" then the smallest element will appear at the top of the priority queue
                    bool operator()(const ConfigAndDistType& c1, const ConfigAndDistType& c2) const
                    {
                        // If both expected distances are the same, then we want to explore the one that has the smaller heuristic distance
                        if (EigenHelpers::IsApprox(c1.second, c2.second, 1e-10))
                        {
                            const double hdist_c1 = heuristic_fn_(c1.first);
                            const double hdist_c2 = heuristic_fn_(c2.first);
                            return (hdist_c1 > hdist_c2);
                        }
                        // If expected distances are different, we want to explore the one with the smaller expected distance
                        else
                        {
                            return (c1.second > c2.second);
                        }
                    }

                    const std::function<double(const ConfigType&)>& heuristic_fn_;
            };

            template<typename ConfigHasher = std::hash<ConfigType>>
            static std::vector<ConfigType, Allocator> ExtractPathBasic(const std::unordered_map<ConfigType, ConfigType, ConfigHasher>& backpointers, ConfigType last_state)
            {
                std::vector<ConfigType, Allocator> path;
                for (auto backpointer_ittr = backpointers.find(last_state); backpointer_ittr != backpointers.end(); backpointer_ittr = backpointers.find(last_state))
                {
                    path.push_back(last_state);
                    last_state = backpointer_ittr->second;
                }
                path.push_back(last_state);
                std::reverse(path.begin(), path.end());
                return path;
            }

        public:
            template<typename ConfigHasher = std::hash<ConfigType>>
            static std::pair<std::vector<ConfigType, Allocator>, std::map<std::string, double>> Plan(
                    const ConfigType& start,
                    const std::function<std::vector<ConfigType, Allocator>(const ConfigType&)>& neighbour_fn,
                    const std::function<double(const ConfigType&, const ConfigType&)>& distance_fn,
                    const std::function<double(const ConfigType&)>& heuristic_fn,
                    const std::function<bool(const ConfigType&)>& goal_reached_fn)
            {
                const auto start_time = std::chrono::steady_clock::now();

//                ros::NodeHandle nh;
//                visualization_msgs::Marker marker;
//                marker.header.frame_id = "mocap_world";
//                marker.type = visualization_msgs::Marker::CUBE_LIST;
//                marker.action = visualization_msgs::Marker::ADD;
//                marker.ns = "explored_states";
//                marker.id = 1;
//                marker.scale.x = ROSHelpers::GetParam(nh, "world_x_step", 1.0);
//                marker.scale.y = ROSHelpers::GetParam(nh, "world_y_step", 1.0);
//                marker.scale.z = ROSHelpers::GetParam(nh, "world_z_step", 1.0);
//                marker.points.reserve(4096);
//                marker.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(1.0, 1.0, 0.0, 1.0);

//                visualization_msgs::Marker neighbours_marker;
//                neighbours_marker.header.frame_id = "mocap_world";
//                neighbours_marker.type = visualization_msgs::Marker::CUBE_LIST;
//                neighbours_marker.action = visualization_msgs::Marker::ADD;
//                neighbours_marker.ns = "neighbours";
//                neighbours_marker.id = 1;
//                neighbours_marker.scale.x = ROSHelpers::GetParam(nh, "world_x_step", 1.0);
//                neighbours_marker.scale.y = ROSHelpers::GetParam(nh, "world_y_step", 1.0);
//                neighbours_marker.scale.z = ROSHelpers::GetParam(nh, "world_z_step", 1.0);
//                neighbours_marker.points.reserve(26);
//                neighbours_marker.color = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>::MakeFromFloatColors(0.0, 0.0, 1.0, 0.3);

//                ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100, false);



                std::pair<std::vector<ConfigType, Allocator>, std::map<std::string, double>> results;

                const AStarComparator astar_compare(heuristic_fn);
                std::priority_queue<ConfigAndDistType, std::vector<ConfigAndDistType>, AStarComparator> frontier(astar_compare);
                std::unordered_set<ConfigType, ConfigHasher> explored;
                std::unordered_map<ConfigType, double, ConfigHasher> cost_to_come;
                std::unordered_map<ConfigType, ConfigType, ConfigHasher> backpointers;

                frontier.push(ConfigAndDistType(start, heuristic_fn(start)));
                cost_to_come[start] = 0.0;

                bool goal_reached = false;
                while (!goal_reached && frontier.size() > 0)
                {
                    const ConfigAndDistType current = frontier.top();
                    frontier.pop();
                    const ConfigType& current_node = current.first;

                    if (goal_reached_fn(current_node) == true)
                    {
                        results.first = ExtractPathBasic(backpointers, current_node);
                        goal_reached = true;
                    }
                    // Double check if we've already explored this node:
                    //    a single node can be inserted into the frontier multiple times at the same or different priorities
                    //    so we want to avoid the expense of re-exploring it, and just discard this one once we pop it
                    else if (explored.find(current_node) == explored.end())
                    {
//                        geometry_msgs::Point p;
//                        p.x = current_node[0];
//                        p.y = current_node[1];
//                        p.z = current_node[2];
//                        marker.points.push_back(p);
//                        marker.header.stamp = ros::Time::now();
//                        marker_pub.publish(marker);

                        explored.insert(current_node);
                        const double current_cost_to_come = cost_to_come.at(current_node);

                        // Expand the node to find all neighbours, adding them to the frontier if we have not already explored them
                        const auto neighbours = neighbour_fn(current_node);

//                        neighbours_marker.points = EigenHelpersConversions::VectorEigenVector3dToVectorGeometryPoint(neighbours);
//                        neighbours_marker.header.stamp = ros::Time::now();
//                        marker_pub.publish(neighbours_marker);
//                        std::cout << std::flush;

                        for (const auto neighbour : neighbours)
                        {
                            // Check if we've already explored this neighbour
                            if (explored.find(neighbour) != explored.end())
                            {
                                continue;
                            }

                            // Do some sanity checks so that we can make assumptions later
                            const auto neighbour_cost_to_come_ittr = cost_to_come.find(neighbour);
                            const auto neighbour_backpointer_ittr = backpointers.find(neighbour);
                            if (neighbour_cost_to_come_ittr == cost_to_come.end())
                            {
                                assert(neighbour_backpointer_ittr == backpointers.end());
                            }
                            if (neighbour_backpointer_ittr == backpointers.end())
                            {
                                assert(neighbour_cost_to_come_ittr == cost_to_come.end());
                            }

                            // If we haven't already explored this neighbour, see if we've found a cheaper path
                            const double neighbour_new_cost_to_come = current_cost_to_come + distance_fn(current_node, neighbour);
                            if (neighbour_cost_to_come_ittr != cost_to_come.end() && neighbour_cost_to_come_ittr->second <= neighbour_new_cost_to_come)
                            {
                                continue;
                            }

                            frontier.push(ConfigAndDistType(neighbour, neighbour_new_cost_to_come + heuristic_fn(neighbour)));
                            cost_to_come[neighbour] = neighbour_new_cost_to_come;
                            backpointers[neighbour] = current_node;
                        }
                    }
                    else
                    {
//                        std::cout << "Already explored this node, skipping\n";
                    }
                }

                const auto end_time = std::chrono::steady_clock::now();
                results.second["planning time"] = std::chrono::duration<double>(end_time - start_time).count();
                results.second["nodes explored"] = explored.size();

                return results;
            }
    };
}

#endif // SIMPLE_ASTAR_PLANNER_HPP
