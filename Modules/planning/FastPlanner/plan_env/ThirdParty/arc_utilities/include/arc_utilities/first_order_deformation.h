#ifndef FIRST_ORDER_DEFORMATION_H
#define FIRST_ORDER_DEFORMATION_H

#include <functional>

namespace arc_utilities
{
    namespace FirstOrderDeformation
    {
        typedef std::pair<ssize_t, ssize_t> ConfigType;
        typedef std::pair<ConfigType, double> ConfigAndDistType;
        typedef std::function<bool(const ssize_t row, const ssize_t col)> ValidityCheckFnType;

        bool CheckFirstOrderDeformation(
                const ssize_t rows,
                const ssize_t cols,
                const ValidityCheckFnType& validity_check_fn);
    }
}

#endif // FIRST_ORDER_DEFORMATION_H
