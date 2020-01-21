#include <iostream>
#include "arc_utilities/shortcut_smoothing.hpp"
#include "arc_utilities/pretty_print.hpp"

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    EigenHelpers::VectorVector3d vec(10);
    for (int i = 0; i < 10; ++i)
    {
        vec[i] = Eigen::Vector3d(i, 0.0, 0.0);
    }

    const double step_size = 0.1;
    const auto collision_fn = [] (const Eigen::Vector3d& location) { (void)location; return location.norm() > 4.0; };

    auto vec_smoothed = shortcut_smoothing::InterpolateWithCollisionCheck(vec, 1, 4, step_size, collision_fn);


    std::cout << "Original:\n" << PrettyPrint::PrettyPrint(vec, true, "\n") << std::endl << std::endl;
    std::cout << "Smoothed:\n" << PrettyPrint::PrettyPrint(vec_smoothed, true, "\n") << std::endl << std::endl;

    return 0;
}
