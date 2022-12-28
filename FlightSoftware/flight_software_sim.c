
#include "flight_software_sim_header.h"
/*
This file contains a function to call to simulate flight software, outputs, states and more
gcc -fPIC -shared -o flight_software_sim.so flight_software_sim.c
*/



double* flight_software_sim(double* sensor_matrix,double time_s, double dt_s)
{
/*
Returns:

Inputs:
    sensor_matrix[3][6] (double) = Current measurment from all sensors (see Simulation/Sensos.py for parsing)
    time_s              (double) = Current time_s in secounds
    dt_s                (double) = Change in time between the last time this function was called in secounds
*/
    sensor_matrix[0] = 69.0;
    return sensor_matrix;
}

