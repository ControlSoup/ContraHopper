#include <stdio.h>

typedef enum{
    INIT      = 0,
    IDLE      = 10,
    LAUNCH    = 20,
    HOVER     = 30,
    LAND      = 40,
    ABORT     = 50,
    TERMINATE = 60
}state_name;


state_name machine_state = INIT;
int main(){
    switch (machine_state){
        case INIT:
        // init
    }
}