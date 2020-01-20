#ifndef LEADLAGFILTER_H
#define LEADLAGFILTER_H

#include <math.h>

using namespace std;


class LeadLagFilter
{
    public:

        //构造函数
        LeadLagFilter()
        {
            T = 0.0f;
            Kd = 0.0f;
            Output_prev = 0.0f;
            Input_prev = 0.0f;
        }

        void set_Time_constant(float Time_constant,float _Kd);

        float apply(float input, float dt);

        // Return the cutoff frequency
        float get_Time_constant() const { return T; }

        float get_Kd() const { return Kd; }

    private:

        //Previous output
        float Output_prev;

        float Input_prev;

        float Kd;

        //Time constant
        float T;
};

float LeadLagFilter::apply(float input, float dt)
{
        // do the filtering
        float output = 1/(T+dt)*(T*Output_prev + input - Input_prev + dt*Kd*input);

        Output_prev = output;

        Input_prev = input;

        // return the value.
        return output;
}

void LeadLagFilter::set_Time_constant(float Time_constant,float _Kd)
{
    T = Time_constant;
    Kd = _Kd;
}

#endif
