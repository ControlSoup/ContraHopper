

void radio_update(float rc_cntrl[5]){
    /*!
        @brief Updates radio reciever readings for each controller
        @param rc_cntrl (float[5]) = [pitch,roll,yaw,throttle,knob]
    */ 
    rc_ctrl[0] = ch1.getValue(); 
    if (rc_ctrl[0] < 1000) rc_ctrl[0] = 1000;
    if (rc_ctrl[0] > 2000) rc_ctrl[0] = 2000;
    rc_ctrl[1] = ch2.getValue(); 
    if (rc_ctrl[1] < 1000) rc_ctrl[1] = 1000;
    if (rc_ctrl[1] > 2000) rc_ctrl[1] = 2000;
    rc_ctrl[2] = ch4.getValue(); 
    if (rc_ctrl[2] < 1000) rc_ctrl[2] = 1000;
    if (rc_ctrl[2] > 2000) rc_ctrl[2] = 2000;
    rc_ctrl[3] = ch3.getValue(); 
    if (rc_ctrl[3] < 1000) rc_ctrl[3] = 1000;
    if (rc_ctrl[3] > 2000) rc_ctrl[3] = 2000;
    rc_ctrl[4] = ch5.getValue(); 
    if (rc_ctrl[4] < 1000) rc_ctrl[4] = 1000;
    if (rc_ctrl[4] > 2000) rc_ctrl[4] = 2000;
    rc_ctrl[5] = ch5.getValue(); 
    if (rc_ctrl[5] < 1000) rc_ctrl[5] = 1000;
    if (rc_ctrl[5] > 2000) rc_ctrl[5] = 2000;
}
