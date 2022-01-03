#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <math.h>

using namespace std;


class LowPassFilter
{
    public:
        //构造函数
        LowPassFilter(void)
        {
            T = 0.0f;
            Output_prev = 0.0f;
        }

        void set_Time_constant(double Time_constant);

        double apply(double input, double dt);

        // Return the cutoff frequency
        double get_Time_constant() const { return T; }

    private:

        //Previous output
        double Output_prev;

        //Time constant
        double T;
};

double LowPassFilter::apply(double input, double dt)
{
        // do the filtering
        double output = T/(T+dt)*Output_prev + dt/(T+dt)*input;

        Output_prev = output;

        // return the value.
        return output;
}

void LowPassFilter::set_Time_constant(double Time_constant)
{
    T = Time_constant;
}


#endif
