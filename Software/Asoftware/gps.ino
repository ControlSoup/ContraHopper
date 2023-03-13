
void gps_setup(){
    /*!
    @brief Intializes the gps sensor
    */

    //myGPS.enableDebugging(); // Uncomment this line to enable debug messages
    if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
    {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }
    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void gps_update(float gps_data[4]){
    /*!
    @brief Updates gps data array, with new senosr data
    @param gps_data (float[4]) = [latitude,longitude,altitude,SIV]
    */
    gps_data[0] = myGPS.getLatitude();
    gps_data[1] = myGPS.getLongitude();
    gps_data[2] = myGPS.getAltitude();
    gps_data[3] = myGPS.getSIV();
}
