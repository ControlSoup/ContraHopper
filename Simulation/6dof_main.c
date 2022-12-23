#include "6dof_head.h"





int main()
{
    /*
    ==============
    Sim Inputs
    ==============
    */

    double
        sim_frequency_hz = 500,
        start_time = 0.0,
        end_time = 0.0,
        dt = 1/sim_frequency_hz;
    
    /*
    ==============
    Initalization
    ==============
    */


    // Make a array that spans start to end times, with increments of dt
    double sim_time_array_s[(end_time - start_time)/dt];
    
        


    return 0;
}   