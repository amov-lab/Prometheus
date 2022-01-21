#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/abb_irb1600_145_fk_fast.hpp>
#include <arc_utilities/iiwa_7_fk_fast.hpp>
#include <arc_utilities/iiwa_14_fk_fast.hpp>
#include <arc_utilities/simple_dtw.hpp>
#include <arc_utilities/simple_prm_planner.hpp>

int main(int argc, char** argv)
{
    printf("%d arguments\n", argc);
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx, argv[idx]);
    }
    std::cout << "Testing PrettyPrints..." << std::endl;
    std::cout << PrettyPrint::PrettyPrint(Eigen::Isometry3d::Identity()) << std::endl;
    std::cout << PrettyPrint::PrettyPrint(Eigen::Vector3d(0.0, 0.0, 0.0)) << std::endl;
    std::cout << PrettyPrint::PrettyPrint(std::vector<bool>{true, false, true, false}) << std::endl;
    std::cout << "...done" << std::endl;
    const std::vector<double> abb_irb_1600_145_base_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const EigenHelpers::VectorIsometry3d abb_irb_1600_145_link_transforms = ABB_IRB1600_145_FK_FAST::GetLinkTransforms(abb_irb_1600_145_base_config);
    std::cout << "ABB IRB1600-145 Link transforms:\n" << PrettyPrint::PrettyPrint(abb_irb_1600_145_link_transforms, false, "\n") << std::endl;
    const std::vector<double> iiwa_7_base_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const EigenHelpers::VectorIsometry3d iiwa_7_link_transforms = IIWA_7_FK_FAST::GetLinkTransforms(iiwa_7_base_config);
    std::cout << "IIWA 7 Link transforms:\n" << PrettyPrint::PrettyPrint(iiwa_7_link_transforms, false, "\n") << std::endl;
    const std::vector<double> iiwa_14_base_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const EigenHelpers::VectorIsometry3d iiwa_14_link_transforms = IIWA_14_FK_FAST::GetLinkTransforms(iiwa_14_base_config);
    std::cout << "IIWA 14 Link transforms:\n" << PrettyPrint::PrettyPrint(iiwa_14_link_transforms, false, "\n") << std::endl;
    std::cout << std::endl;

    // Test Vector3d averaging
    EigenHelpers::VectorVector3d testvecs(8, Eigen::Vector3d::Zero());
    testvecs[0] = Eigen::Vector3d(-1.0, -1.0, -1.0);
    testvecs[1] = Eigen::Vector3d(-1.0, -1.0, 1.0);
    testvecs[2] = Eigen::Vector3d(-1.0, 1.0, -1.0);
    testvecs[3] = Eigen::Vector3d(-1.0, 1.0, 1.0);
    testvecs[4] = Eigen::Vector3d(1.0, -1.0, -1.0);
    testvecs[5] = Eigen::Vector3d(1.0, -1.0, 1.0);
    testvecs[6] = Eigen::Vector3d(1.0, 1.0, -1.0);
    testvecs[7] = Eigen::Vector3d(1.0, 1.0, 1.0);
    std::cout << "Individual vectors:\n" << PrettyPrint::PrettyPrint(testvecs, true, "\n") << std::endl;
    Eigen::Vector3d averagevec = EigenHelpers::AverageEigenVector3d(testvecs);
    std::cout << "Average vector: " << PrettyPrint::PrettyPrint(averagevec) << std::endl;
    std::cout << std::endl;
    assert(averagevec.isMuchSmallerThan(10^-10) && "The result of this average should be (0, 0, 0)");


    // Test weighted dot product functions
    Eigen::Vector3d weights(1.0, 2.0, 3.0);
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[0]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[0], weights) << std::endl;
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[7]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[7], weights) << std::endl;
    std::cout << "Weighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[0], testvecs[7], weights) << std::endl;
    std::cout << "Unweighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[0], testvecs[7], Eigen::Vector3d::Ones()) << std::endl;
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[1]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[1], weights) << std::endl;
    std::cout << "Vector: " << PrettyPrint::PrettyPrint(testvecs[2]) << " Weighted norm: " << EigenHelpers::WeightedNorm(testvecs[2], weights) << std::endl;
    std::cout << "Weighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[1], testvecs[2], weights) << std::endl;
    std::cout << "Unweighted angle between vectors: " << EigenHelpers::WeightedAngleBetweenVectors(testvecs[1], testvecs[2], Eigen::Vector3d::Ones()) << std::endl;
    std::cout << std::endl;

    // Test truncated normal distribution
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937_64 prng(seed);
    arc_helpers::TruncatedNormalDistribution dist(0.0, 1.0, -5.0, 5.0);
    std::vector<double> test_trunc_normals(1000, 0.0);
    for (size_t idx = 0; idx < test_trunc_normals.size(); idx++)
    {
        test_trunc_normals[idx] = dist(prng);
    }
    std::cout << "Truncated normal test:\n" << PrettyPrint::PrettyPrint(test_trunc_normals, false, ",") << std::endl;
    std::cout << std::endl;

    // Test multivariate gaussian
    std::cout << "MVN Gaussian test:\n";
    Eigen::Vector2d mean(0.0, 1.0);
    Eigen::Matrix2d covar;
    covar << 10.0, 5.0,
             5.0, 10.0;
    arc_helpers::MultivariteGaussianDistribution mvn_dist(mean, covar);
    std::vector<Eigen::VectorXd> mvn_gaussians(3000);
    for (size_t idx = 0; idx < mvn_gaussians.size(); idx++)
    {
        mvn_gaussians[idx] = mvn_dist(prng);
        std::cout << mvn_gaussians[idx].transpose() << std::endl;
    }
    std::cout << std::endl;

    // Test DTW
    std::vector<double> reversed_test_trunc_normals = test_trunc_normals;
    std::reverse(reversed_test_trunc_normals.begin(), reversed_test_trunc_normals.end());
    std::function<double(const double&, const double&)> double_distance_fn = [] (const double& d1, const double& d2) { return std::abs(d1 - d2); };
    const double test_dist = simple_dtw::ComputeDTWDistance(test_trunc_normals, test_trunc_normals, double_distance_fn);
    const double reversed_test_dist = simple_dtw::ComputeDTWDistance(test_trunc_normals, reversed_test_trunc_normals, double_distance_fn);
    std::cout << "DTW distance test = " << test_dist << ", reversed = " << reversed_test_dist << std::endl;
    std::cout << std::endl;
}
