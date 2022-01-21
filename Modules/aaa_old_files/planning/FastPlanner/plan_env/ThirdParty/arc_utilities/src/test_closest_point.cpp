#include "arc_utilities/eigen_helpers.hpp"
#include "arc_utilities/pretty_print.hpp"

using namespace Eigen;
using namespace EigenHelpers;

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const Vector3d base_point(-0.2, 0.035, 0.2055);
    const Vector3d normal(0, 0, 1);

    const Vector3d test_point(-0.198425, 0.0342693, 0.0417458);

    const auto distances = DistanceToLine(base_point, normal, test_point);

    std::cout << "Distance to line: " << distances.first << " DISPLACEMENT along line (can be negative): " << distances.second << std::endl;

    const Vector3d point_on_line = base_point + distances.second * normal;

    std::cout << "Point on line: " << point_on_line.transpose() << std::endl;
    std::cout << "Dot product: " << normal.dot(test_point - point_on_line) << std::endl;

    return 0;
}
