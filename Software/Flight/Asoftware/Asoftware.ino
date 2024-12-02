#include "network_outputs.h"

float timer = 0.0;

typedef struct{
    float curr_time;
    float prev_time;
    const float target_dt;
    float dt;
}LoopTimer;

void loop_timer_enforce_dt(LoopTimer *timer){
    
    // Stall until time is acheieved
    timer->curr_time = millis() / 1000.0;
    timer->dt = timer->curr_time - timer->prev_time;
    while(timer->dt < timer->target_dt){
        timer->curr_time = millis() / 1000.0;
        timer->dt = timer->curr_time - timer->prev_time;
    }

    timer->prev_time = timer->curr_time;
}

typedef enum{
    STATE_INIT = 0,
    STATE_IDLE = 1,
    STATE_RUN = 2,
    STATE_ABORT = 1000
} States;

LoopTimer GlobalTimer = {.curr_time = 0, .prev_time = 0, .target_dt = 0.01, .dt = 0.0};
States curr_state = STATE_INIT;
NWI curr_inputs = NWI_UNKNOWN;

void setup(){

    Serial.begin(115200);
    setup_udp();
    curr_state = STATE_IDLE;
}

void loop(){
    Serial.println(curr_inputs);

    // Get inputs from network
    if (timer >= 0.1){
        curr_inputs = recieve_udp();
        timer = 0.0;
    } 
    timer += GlobalTimer.dt;

    // Always ABORT
    if (curr_inputs == NWI_ABORT){
        curr_state = STATE_ABORT;
    }

    // Update outputs 
    NWO_time__s = GlobalTimer.curr_time;
    NWO_state__ = float(curr_state);

    switch (curr_state){

        case STATE_IDLE:
            switch(curr_inputs){
                case NWI_START:
                    curr_state = STATE_RUN;
                    break;
            }
            break;

        case STATE_RUN:
            switch(curr_inputs){
                case NWI_STOP:
                    curr_state = STATE_IDLE; 
                    break;
            }
            break;

        case STATE_ABORT:
            curr_state = STATE_IDLE;
            break;

        default:
            Serial.printf("This is a poinsed arrow...state: %d\n", curr_state);
            curr_state = STATE_IDLE;
            break;
    }

    // Send data
    send_udp();
    loop_timer_enforce_dt(&GlobalTimer);
}
