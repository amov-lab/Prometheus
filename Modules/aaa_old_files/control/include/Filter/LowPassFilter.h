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

        void set_Time_constant(float Time_constant);

        float apply(float input, float dt);

        // Return the cutoff frequency
        float get_Time_constant() const { return T; }

    private:

        //Previous output
        float Output_prev;

        //Time constant
        float T;
};

float LowPassFilter::apply(float input, float dt)
{
        // do the filtering
        float output = T/(T+dt)*Output_prev + dt/(T+dt)*input;

        Output_prev = output;

        // return the value.
        return output;
}

void LowPassFilter::set_Time_constant(float Time_constant)
{
    T = Time_constant;
}


#endif
