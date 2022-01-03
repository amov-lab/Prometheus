#ifndef HIGHPASSFILTER_H
#define HIGHPASSFILTER_H

#include <math.h>

using namespace std;


class HighPassFilter
{
    public:

        //构造函数
        HighPassFilter(void)
        {
            T = 0.0f;
            Output_prev = 0.0f;
            Input_prev = 0.0f;
        }

        void set_Time_constant(double Time_constant);

        double apply(double input, double dt);

        // Return the cutoff frequency
        double get_Time_constant() const { return T; }

    private:

        //Previous output
        double Output_prev;

        double Input_prev;

        //Time constant
        double T;
};

double HighPassFilter::apply(double input, double dt)
{
        // do the filtering
        double output = 1/(T+dt)*(T*Output_prev + input - Input_prev);

        Output_prev = output;

        Input_prev = input;

        // return the value.
        return output;
}

void HighPassFilter::set_Time_constant(double Time_constant)
{
    T = Time_constant;
}

#endif
