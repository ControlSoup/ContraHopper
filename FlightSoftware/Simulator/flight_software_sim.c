
#include "flight_software_sim_header.h"
/*
This file contains a function to call to simulate flight software, outputs, states and more
gcc -fPIC -shared -o flight_software_sim.so flight_software_sim.c
*/

typedef struct{
    double kp;
    double ki;
    double kd;
    double pid_I;
    double last_error
}pid_storage;



double* control(double target,double actual, pid_storage pid_positon, double dt){

     // Calculate error
    double error = target - actual;

    // Update I term
    pid_positon.pid_I += pid_positon.ki * error * dt;

    // Store output
    double* pid_output = {0,0,0};
    pid_output[0] = (pid_positon.kp * error) + 
                    (pid_positon.pid_I) +  
                    ((pid_positon.last_error-error)* pid_positon.kd)/dt;
    return pid_output;
}

double* flight_software_sim(double* sensor_vector,double* input_vector,double time_s, double dt_s)
{
/*
Returns:

Inputs:
    sensor_matrix[3][6] (double) = Current measurment from all sensors (see Simulation/Sensos.py for parsing)
    time_s              (double) = Current time_s in secounds
    dt_s                (double) = Change in time between the last time this function was called in secounds
*/
    pid_storage position_pid = {1.0,0.0,0.0,0.0,0.0};
    input_vector = control(1.0,sensor_vector[0],position_pid,1.0);
    return input_vector;
}
