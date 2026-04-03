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

        void set_Time_constant(double Time_constant,double _Kd);

        double apply(double input, double dt);

        // Return the cutoff frequency
        double get_Time_constant() const { return T; }

        double get_Kd() const { return Kd; }

    private:

        //Previous output
        double Output_prev;

        double Input_prev;

        double Kd;

        //Time constant
        double T;
};

double LeadLagFilter::apply(double input, double dt)
{
        // do the filtering
        double output = 1/(T+dt)*(T*Output_prev + input - Input_prev + dt*Kd*input);

        Output_prev = output;

        Input_prev = input;

        // return the value.
        return output;
}

void LeadLagFilter::set_Time_constant(double Time_constant,double _Kd)
{
    T = Time_constant;
    Kd = _Kd;
}

#endif
