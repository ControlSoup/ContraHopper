
#include "flight_software_sim_header.h"
/*
File currently
*/


#define Kp         0
#define Ki         1
#define Kd         2
#define pid_I      3
#define last_error 4




double PID(double target,double actual, double pid_storage[4], double dt){
    /*
    Returns:
        pid_output (double) = output of a pid controller
    */
     // Calculate error
    double error = target - actual;

    // Update I term
    pid_storage[pid_I] += pid_storage[Ki] * error * dt;

    // Calculate output
    double pid_output = (pid_storage[Kp] * error) + 
                        (pid_storage[pid_I]) +  
                        ((pid_storage[last_error] - error) * pid_storage[Kd]) / dt;
    
    // Update last error
    pid_storage[last_error] = error;
    return pid_output;
}


double* control(double estimate_state_vector[18], 
                double target_state_vector[18],
                double pid_storage[12],
                double dt){
    /*
    Returns:
     output_vector (double(6)+double(3)) = the desired forces and moments to acheive the desired state
     [Fx,Fy,Fz,Mx,My,Mz,
     pid_position_x_I,pid_position_x_last_error,
     pid_position_y_I,pid_position_y_last_error,
     pid_position_z_I,pid_position_z_last_error]
    */

    // Parse estimate_state_vector
    double estimated_position_m[3] = {estimate_state_vector[0],estimate_state_vector[1],estimate_state_vector[2]};

    // Parse target_state_vector
    double target_position_m[3]    = {target_state_vector[0],target_state_vector[1],target_state_vector[2]};

    // Parse pid_storage
    double z_position_pid[4]       = {pid_storage[8],pid_storage[9],pid_storage[10],pid_storage[11]};
    // Z position controller
    double z_pid_result            = PID(target_position_m[2],estimated_position_m[2],z_position_pid,dt);

    double* output_vector;
    output_vector[0] = 0;
    output_vector[1] = 0;
    output_vector[2] = z_pid_result;
    output_vector[3] = 0;
    output_vector[4] = 0;
    output_vector[5] = 0;   
    output_vector[6] = pid_storage[6];
    output_vector[7] = pid_storage[7];
    output_vector[8] = pid_storage[8];
    output_vector[9] = pid_storage[9];
    output_vector[10] = pid_storage[10];
    output_vector[11] = pid_storage[11];

    int i;     
    for (i = 0; i < 12; i++) printf("%lf",output_vector[i]);
    return output_vector;
}