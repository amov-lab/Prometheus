#include "arc_utilities/timing.hpp"

double arc_utilities::GlobalStopwatch(const StopwatchControl control)
{
    static arc_utilities::Stopwatch global_stopwatch;
    return global_stopwatch(control);
}
