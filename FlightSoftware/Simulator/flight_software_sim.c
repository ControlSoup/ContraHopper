
#include "flight_software_sim_header.h"
/*
File currently
*/

#define Kp         0
#define Ki         1
#define Kd         2
#define pid_I      3
#define last_error 4

//generate a .so
//gcc -fPIC -shared -o c_test.so c_test.c 

double* PID(double target,double actual, double* pid_storage, double dt){
    /*
    Returns:
        pid_output (double) = output of a pid controller
    */
     // Calculate error
    double error      = target - actual;

    // Update I term
    double I_term      = pid_storage[pid_I] + pid_storage[Ki] * error * dt;

    // Calculate output
    double pid_result = (pid_storage[Kp] * error) + 
                        (pid_storage[pid_I]) +  
                        ((pid_storage[last_error] - error) * pid_storage[Kd]) / dt;

    double* pid_output;
    pid_output[0] = pid_result;
    pid_output[1] = I_term;
    pid_output[2] = error;
    return pid_output;
}
