#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <iostream>
#include <fstream>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/simple_hierarchical_clustering.hpp>
#include <arc_utilities/abb_irb1600_145_fk_fast.hpp>

int main(int argc, char** argv)
{
    printf("%d arguments\n", argc);
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx, argv[idx]);
    }
    const size_t num_points = (argc >= 2) ? (size_t)(atoi(argv[1])) : 1000;
    std::cout << "Generating " << num_points << " random points..." << std::endl;
    const auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937_64 prng(seed);
    std::uniform_real_distribution<double> dist(0.0, 10.0);
    EigenHelpers::VectorVector3d random_points(num_points);
    std::vector<size_t> indices(num_points);
    for (size_t idx = 0; idx < num_points; idx++)
    {
        const double x = dist(prng);
        const double y = dist(prng);
        random_points[idx] = Eigen::Vector3d(x, y, 0.0);
        indices[idx] = idx;
    }
    std::cout << "Clustering " << num_points << " points..." << std::endl;
    std::function<double(const Eigen::Vector3d&, const Eigen::Vector3d&)> distance_fn = [] (const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) { return EigenHelpers::Distance(v1, v2); };
    const Eigen::MatrixXd distance_matrix = arc_helpers::BuildDistanceMatrix(random_points, distance_fn);
    const std::vector<std::vector<size_t>> clusters = simple_hierarchical_clustering::SimpleHierarchicalClustering::Cluster(indices, distance_matrix, 1.0).first;
    for (size_t cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++)
    {
        const std::vector<size_t>& current_cluster = clusters[cluster_idx];
        const double cluster_num = 1.0 + (double)cluster_idx;
        for (size_t element_idx = 0; element_idx < current_cluster.size(); element_idx++)
        {
            const size_t index = current_cluster[element_idx];
            random_points[index].z() = cluster_num;
        }
    }
    std::cout << "Saving to CSV..." << std::endl;
    const std::string log_file_name = (argc >=3 ) ? std::string(argv[2]) : "/tmp/test_hierarchical_clustering.csv";
    std::ofstream log_file(log_file_name, std::ios_base::out);
    if (!log_file.is_open())
    {
        std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << log_file_name << "\x1b[0m \n";
        throw std::invalid_argument("Log filename must be write-openable");
    }
    for (size_t idx = 0; idx < num_points; idx++)
    {
        const Eigen::Vector3d& point = random_points[idx];
        log_file << point.x() << "," << point.y() << "," << point.z() << std::endl;
    }
    log_file.close();
    std::cout << "Done saving, you can import into matlab and draw with \"scatter3(x, y, z, 50, z, '.')\"" << std::endl;
    return 0;
}
