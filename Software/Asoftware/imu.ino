
void imu_setup(){
    if (!ism330dhcx.begin_I2C()) {
        Serial.println("Failed to find ISM330DHCX chip");
        while (1) {
            delay(10);
        }
    }

    ism330dhcx.setAccelDataRate(LSM6DS_RATE_833_HZ);
    ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    ism330dhcx.setGyroDataRate(LSM6DS_RATE_833_HZ);
    ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
}

void imu_update(float imu_data[5]){
    /*!
        @brief Updates the imu data array with new sensor data
        @param imu_data (float[6]) = [acceleration_x,acceleration_y,acceleration_z,
                                          gyroscope_x,   gyroscope_y,   gyroscope_z,
                                                                        temperature]
    */
    sensors_event_t event_accelerometer;
    sensors_event_t event_gyroscope;
    sensors_event_t event_temperature;
    ism330dhcx.getEvent(&event_accelerometer, &event_gyroscope, &event_temperature);
     imu_data[0] = event_accelerometer.acceleration.x;
    imu_data[1] = event_accelerometer.acceleration.y;
    imu_data[2] = event_accelerometer.acceleration.z;
    imu_data[3] = event_gyroscope.gyro.x;
    imu_data[4] = event_gyroscope.gyro.y;
    imu_data[5] = event_gyroscope.gyro.z;
    imu_data[6] = event_temperature.temperature;    
}

void print_imu(float imu_data[5]){
    Serial.print(imu_data[0]);
    Serial.print(" ");
    Serial.print(imu_data[1]);
    Serial.print(" ");
    Serial.print(imu_data[2]);
    Serial.print(" ");
    Serial.print(imu_data[3]);
    Serial.print(" ");
    Serial.print(imu_data[4]);
    Serial.print(" ");
    Serial.println(imu_data[5]);
}
