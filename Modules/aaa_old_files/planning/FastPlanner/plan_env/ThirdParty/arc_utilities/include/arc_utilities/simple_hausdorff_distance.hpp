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

#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
    #include <omp.h>
#endif

#ifndef SIMPLE_HAUSDORFF_DISTANCE_HPP
#define SIMPLE_HAUSDORFF_DISTANCE_HPP

namespace simple_hausdorff_distance
{
    class SimpleHausdorffDistance
    {
    private:

        SimpleHausdorffDistance() {}

        static inline size_t GetNumOMPThreads(void)
        {
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
        #if defined(_OPENMP)
            size_t num_threads = 0;
            #pragma omp parallel
            {
                num_threads = (size_t)omp_get_num_threads();
            }
            return num_threads;
        #else
            return 1;
        #endif
#else
            return 1;
#endif
        }

    public:

        template<typename FirstDatatype, typename SecondDatatype, typename FirstAllocator=std::allocator<FirstDatatype>, typename SecondAllocator=std::allocator<SecondDatatype>>
        static double ComputeDistance(const std::vector<FirstDatatype, FirstAllocator>& first_distribution, const std::vector<SecondDatatype, SecondAllocator>& second_distribution, const std::function<double(const FirstDatatype&, const SecondDatatype&)>& distance_fn)
        {
            // Compute the Hausdorff distance - the "maximum minimum" distance
            std::vector<double> per_thread_storage(GetNumOMPThreads(), 0.0);
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
            #pragma omp parallel for
#endif
            for (size_t idx = 0; idx < first_distribution.size(); idx++)
            {
                const FirstDatatype& first = first_distribution[idx];
                double minimum_distance = INFINITY;
                for (size_t jdx = 0; jdx < second_distribution.size(); jdx++)
                {
                    const SecondDatatype& second = second_distribution[jdx];
                    const double current_distance = distance_fn(first, second);
                    if (current_distance < minimum_distance)
                    {
                        minimum_distance = current_distance;
                    }
                }
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
                #if defined(_OPENMP)
                const size_t current_thread_id = (size_t)omp_get_thread_num();
                #else
                const size_t current_thread_id = 0;
                #endif
#else
                const size_t current_thread_id = 0;
#endif
                if (minimum_distance > per_thread_storage[current_thread_id])
                {
                    per_thread_storage[current_thread_id] = minimum_distance;
                }
            }
            double maximum_minimum_distance = 0.0;
            for (size_t idx = 0; idx < per_thread_storage.size(); idx++)
            {
                const double temp_minimum_distance = per_thread_storage[idx];
                if (temp_minimum_distance > maximum_minimum_distance)
                {
                    maximum_minimum_distance = temp_minimum_distance;
                }
            }
            return maximum_minimum_distance;
        }
    };
}
#endif // SIMPLE_HAUSDORFF_DISTANCE_HPP
