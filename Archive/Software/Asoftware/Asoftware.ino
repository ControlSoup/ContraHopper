/*
===========================
Includes
===========================
*/

#include <Adafruit_ISM330DHCX.h>
#include "PWM.hpp" // PWM Managment
#include <Wire.h> // I2C helper
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS

Adafruit_ISM330DHCX ism330dhcx; // IMU
SFE_UBLOX_GPS myGPS; // GPS

/*
===========================
Channels
===========================
*/

PWM 
    ch1(24),
    ch2(12),
    ch3(11),
    ch4(10),
    ch5(9),
    ch5(9);

/*
===========================
Global Variables
===========================
*/

float 
    rc_ctrl[5],
    imu_data[5],
    gps_data[4];

/*
===========================
Setup
===========================
*/

void setup(){
    Wire.begin();
    Serial.begin(57600); 
    delay(50);

    imu_setup();
    gps_setup();

    // Reciever Setup
    ch1.begin(true); 
    ch2.begin(true); 
    ch3.begin(true);
    ch4.begin(true);  
    ch5.begin(true); 
    ch6.begin(true); 
}

void loop(){
    radio_update(rc_ctrl);
    imu_update(imu_data);
    gps_update(gps_data);
    
    print_imu(imu_data);
    
    delay(10);
}
