#include "arc_utilities/eigen_helpers.hpp"
#include "arc_utilities/pretty_print.hpp"

using namespace Eigen;
using namespace EigenHelpers;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const std::vector<double> weights = {1.0, 2.0, 3.0};

    VectorVector3d test_vectors(3);
    test_vectors[0] = Vector3d::UnitX();
    test_vectors[1] = Vector3d::UnitY();
    test_vectors[2] = Vector3d::UnitZ();

    const Vector3d weighted_average = AverageEigenVector(test_vectors, weights);
    std::cout << "Test weights: " << PrettyPrint::PrettyPrint(weights) << std::endl;
    std::cout << "Test vectors:\n" << PrettyPrint::PrettyPrint(test_vectors, true, "\n") << std::endl;
    std::cout << "Weighted Average:\n" << PrettyPrint::PrettyPrint(weighted_average) << std::endl << std::endl;
    assert(weighted_average.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0) / 6.0));


    VectorVector3d repeated_test_vectors;
    for (size_t idx = 0; idx < test_vectors.size(); ++idx)
    {
        repeated_test_vectors.insert(repeated_test_vectors.end(), (size_t)weights[idx], test_vectors[idx]);
    }

    const Vector3d repeated_average = AverageEigenVector(repeated_test_vectors);
    std::cout << "Repeated vectors:\n" << PrettyPrint::PrettyPrint(repeated_test_vectors, true, "\n") << std::endl;
    std::cout << "Repeated Average:\n" << PrettyPrint::PrettyPrint(repeated_average) << std::endl << std::endl;;
    assert(repeated_average.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0) / 6.0));



    const std::vector<double> all_zero_weights = {0.0, 0.0, 0.0};
    const Vector3d weighted_average_all_zero_weights = AverageEigenVector(test_vectors, all_zero_weights);
    std::cout << "Test weights: " << PrettyPrint::PrettyPrint(all_zero_weights) << std::endl;
    std::cout << "Test vectors:\n" << PrettyPrint::PrettyPrint(test_vectors, true, "\n") << std::endl;
    std::cout << "All Zero Weights Average:\n" << PrettyPrint::PrettyPrint(weighted_average_all_zero_weights) << std::endl << std::endl;
    assert(weighted_average_all_zero_weights.isMuchSmallerThan(10^-10) && "The result of this average should be (0, 0, 0)");

    return 0;
}
