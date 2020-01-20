#ifndef ARC_UTILITIES_TIMING_HPP
#define ARC_UTILITIES_TIMING_HPP

#include <chrono>

namespace arc_utilities
{
    enum StopwatchControl {RESET, READ};

    class Stopwatch
    {
        public:
            Stopwatch()
                : start_time_(std::chrono::steady_clock::now())
            {}

            double operator() (const StopwatchControl control = READ)
            {
                const auto end_time = std::chrono::steady_clock::now();
                if (control == RESET)
                {
                    start_time_ = end_time;
                }

                return std::chrono::duration<double>(end_time - start_time_).count();
            }

        private:
            std::chrono::steady_clock::time_point start_time_;
    };

    double GlobalStopwatch(const StopwatchControl control = READ);
}

#endif // ARC_UTILITIES_TIMING_HPP
