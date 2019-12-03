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

        void set_Time_constant(float Time_constant);

        float apply(float input, float dt);

        // Return the cutoff frequency
        float get_Time_constant() const { return T; }

    private:

        //Previous output
        float Output_prev;

        float Input_prev;

        //Time constant
        float T;
};

float HighPassFilter::apply(float input, float dt)
{
        // do the filtering
        float output = 1/(T+dt)*(T*Output_prev + input - Input_prev);

        Output_prev = output;

        Input_prev = input;

        // return the value.
        return output;
}

void HighPassFilter::set_Time_constant(float Time_constant)
{
    T = Time_constant;
}

#endif
