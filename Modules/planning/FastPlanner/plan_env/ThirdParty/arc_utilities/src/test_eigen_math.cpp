#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/aligned_eigen_types.hpp>
#include <arc_utilities/pretty_print.hpp>

inline void TestVector3d(const ssize_t iterations, const Eigen::Isometry3d& base_transform, Eigen::MatrixXd& results)
{
    std::chrono::time_point<std::chrono::steady_clock> vector3_start_time = std::chrono::steady_clock::now();
    for (ssize_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Vector3d test_vector(1.0 * (double)idx, 2.0 * (double)idx, 3.0 * (double)idx);
        results.block<3, 1>(idx * 4, 0) = base_transform * test_vector;
    }
    std::chrono::time_point<std::chrono::steady_clock> vector3_end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> vector3_test_time(vector3_end_time - vector3_start_time);
    std::cout << "Isometry3d * Vector3d test - " << vector3_test_time.count() << "s for " << iterations << " iterations to produce " << results.block<4, 1>(100000, 0) << std::endl;
}

inline void TestVector4d(const ssize_t iterations, const Eigen::Isometry3d& base_transform, Eigen::MatrixXd& results)
{
    std::chrono::time_point<std::chrono::steady_clock> vector4_start_time = std::chrono::steady_clock::now();
    for (ssize_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Vector4d test_vector(1.0 * (double)idx, 2.0 * (double)idx, 3.0 * (double)idx, 1.0);
        results.block<4, 1>(idx * 4, 0) = base_transform * test_vector;
    }
    std::chrono::time_point<std::chrono::steady_clock> vector4_end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> vector4_test_time(vector4_end_time - vector4_start_time);
    std::cout << "Isometry3d * Vector4d test - " << vector4_test_time.count() << "s for " << iterations << " iterations to produce " << results.block<4, 1>(100000, 0) << std::endl;
}

inline void TestAlignedVector3d(const ssize_t iterations, const Eigen::Isometry3d& base_transform, Eigen::MatrixXd& results)
{
    std::chrono::time_point<std::chrono::steady_clock> alignedvector3_start_time = std::chrono::steady_clock::now();
    for (ssize_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Aligned4Vector3<double> test_vector(1.0 * (double)idx, 2.0 * (double)idx, 3.0 * (double)idx);
        results.block<3, 1>(idx * 4, 0) = base_transform * test_vector;
    }
    std::chrono::time_point<std::chrono::steady_clock> alignedvector3_end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> alignedvector3_test_time(alignedvector3_end_time - alignedvector3_start_time);
    std::cout << "Isometry3d * AlignedVector3d test - " << alignedvector3_test_time.count() << "s for " << iterations << " iterations to produce " << results.block<4, 1>(100000, 0) << std::endl;
}

inline void TestManual(const ssize_t iterations, const Eigen::Isometry3d& base_transform, Eigen::MatrixXd& results)
{
    const Eigen::Matrix4d& base_transform_matrix = base_transform.matrix();
    std::chrono::time_point<std::chrono::steady_clock> manual_start_time = std::chrono::steady_clock::now();
    for (ssize_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Vector4d test_vector(1.0 * (double)idx, 2.0 * (double)idx, 3.0 * (double)idx, 1.0);
        results.block<4, 1>(idx * 4, 0) = base_transform_matrix * test_vector;
    }
    std::chrono::time_point<std::chrono::steady_clock> manual_end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> manual_test_time(manual_end_time - manual_start_time);
    std::cout << "Matrix4d * Vector4d test - " << manual_test_time.count() << "s for " << iterations << " iterations to produce " << results.block<4, 1>(100000, 0) << std::endl;
}

int main(int argc, char** argv)
{
    printf("%d arguments\n", argc);
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx, argv[idx]);
    }
    const ssize_t iterations = 100000000;
    const Eigen::Isometry3d base_transform = Eigen::Translation3d(10.0, 10.0, 10.0) * EigenHelpers::QuaternionFromRPY(0.25, 0.5, 0.75);
    Eigen::MatrixXd results = Eigen::MatrixXd::Ones(iterations * 4, 1);
    TestVector3d(iterations, base_transform, results);
    TestVector3d(iterations, base_transform, results);
    TestVector3d(iterations, base_transform, results);
    TestVector3d(iterations, base_transform, results);
    TestVector3d(iterations, base_transform, results);
    //
    TestVector4d(iterations, base_transform, results);
    TestVector4d(iterations, base_transform, results);
    TestVector4d(iterations, base_transform, results);
    TestVector4d(iterations, base_transform, results);
    TestVector4d(iterations, base_transform, results);
    //
    TestAlignedVector3d(iterations, base_transform, results);
    TestAlignedVector3d(iterations, base_transform, results);
    TestAlignedVector3d(iterations, base_transform, results);
    TestAlignedVector3d(iterations, base_transform, results);
    TestAlignedVector3d(iterations, base_transform, results);
    //
    TestManual(iterations, base_transform, results);
    TestManual(iterations, base_transform, results);
    TestManual(iterations, base_transform, results);
    TestManual(iterations, base_transform, results);
    TestManual(iterations, base_transform, results);
    return 0;
}
