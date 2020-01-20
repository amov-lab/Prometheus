#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <mutex>
#include <Eigen/Geometry>

#ifndef SIMPLE_DTW_HPP
#define SIMPLE_DTW_HPP

namespace simple_dtw
{
    template<typename FirstDatatype, typename SecondDatatype, typename DistanceFn,
             typename FirstAllocator = std::allocator<FirstDatatype>,
             typename SecondAllocator = std::allocator<SecondDatatype>>
    class SimpleDTW
    {
    private:

        void InitializeMatrix(const size_t first_sequence_size, const size_t second_sequence_size)
        {
            const ssize_t rows = (ssize_t)first_sequence_size + 1;
            const ssize_t cols = (ssize_t)second_sequence_size + 1;
            if (dtw_matrix_.rows() < rows || dtw_matrix_.cols() < cols)
            {
                dtw_matrix_ = Eigen::MatrixXd::Zero(rows, cols);
                if (rows > 1 && cols > 1)
                {
                    for (ssize_t row = 1; row < rows; row++)
                    {
                        dtw_matrix_(row, 0) = std::numeric_limits<double>::infinity();
                    }
                    for (ssize_t col = 1; col < cols; col++)
                    {
                        dtw_matrix_(0, col) = std::numeric_limits<double>::infinity();
                    }
                }
            }

        }

        Eigen::MatrixXd dtw_matrix_;

    public:

        SimpleDTW()
        {
            InitializeMatrix(0, 0);
        }

        SimpleDTW(const size_t first_sequence_size, const size_t second_sequence_size)
        {
            InitializeMatrix(first_sequence_size, second_sequence_size);
        }

        double EvaluateWarpingCost(
                const std::vector<FirstDatatype, FirstAllocator>& first_sequence,
                const std::vector<SecondDatatype, SecondAllocator>& second_sequence,
                const DistanceFn& distance_fn)
        {
            InitializeMatrix(first_sequence.size(), second_sequence.size());
            //Compute DTW cost for the two sequences
            for (ssize_t i = 1; i <= (ssize_t)first_sequence.size(); i++)
            {
                const FirstDatatype& first_item = first_sequence[(size_t)i - 1];
                for (ssize_t j = 1; j <= (ssize_t)second_sequence.size(); j++)
                {
                    const SecondDatatype& second_item = second_sequence[(size_t)j - 1];
                    const double index_cost = distance_fn(first_item, second_item);
                    double prev_cost = 0.0;
                    // Get the three neighboring values from the matrix to use for the update
                    double im1j = dtw_matrix_(i - 1, j);
                    double im1jm1 = dtw_matrix_(i - 1, j - 1);
                    double ijm1 = dtw_matrix_(i, j - 1);
                    // Start the update step
                    if (im1j < im1jm1 && im1j < ijm1)
                    {
                        prev_cost = im1j;
                    }
                    else if (ijm1 < im1j && ijm1 < im1jm1)
                    {
                        prev_cost = ijm1;
                    }
                    else
                    {
                        prev_cost = im1jm1;
                    }
                    // Update the value in the matrix
                    const double new_cost = index_cost + prev_cost;
                    dtw_matrix_(i, j) = new_cost;
                }
            }
            //Return total path cost
            const double warping_cost = dtw_matrix_((ssize_t)first_sequence.size(), (ssize_t)second_sequence.size());
            return warping_cost;
        }
    };

    // DistanceFn must match the prototype std::function<double(const FirstDataype&, const SecondDatatype&)>
    template<typename FirstDatatype, typename SecondDatatype, typename DistanceFn,
             typename FirstAllocator = std::allocator<FirstDatatype>,
             typename SecondAllocator = std::allocator<SecondDatatype>>
    inline double ComputeDTWDistance(
            const std::vector<FirstDatatype, FirstAllocator>& first_sequence,
            const std::vector<SecondDatatype, SecondAllocator>& second_sequence,
            const DistanceFn& distance_fn)
    {
        SimpleDTW<FirstDatatype, SecondDatatype, DistanceFn, FirstAllocator, SecondAllocator> dtw_evaluator;
        return dtw_evaluator.EvaluateWarpingCost(first_sequence, second_sequence, distance_fn);
    }
}

#endif // SIMPLE_DTW_HPP
