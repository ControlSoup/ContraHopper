
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

void control(double state_vector[6],double target_vector[6], double control_output[6]){

    int i;

    for (i =0; i < 6; i++){
        control_output[i] = state_vector[i] + target_vector[i];
    }

}
