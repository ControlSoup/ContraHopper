import numpy as np
from numpy.random import randn


"""
===========================
Supporting function and constants
===========================
"""
mps2_2_lsb  = 4096
radps_2_lsb = 1.143191 # (lsb2deg * deg2rad)

def add_noise(mean,variance):
    # Returns a random data point based on gaussian inputs
    return mean + randn()*(np.sqrt(variance))

"""
===========================
Classes
===========================
"""

class Sensors:
    '''
    Stores measurments in vector and variable form.
    '''
    def __init__(self,
                 accel_mps2 = np.zeros(3),
                 gyro_radps = np.zeros(3)):
                 self.accel_lsb       = accel_mps2 * mps2_2_lsb
                 self.gyro_lsb        = gyro_radps * radps_2_lsb 
                 self.sensor_vector   = np.array([self.accel_lsb,self.gyro_lsb]).flatten()
    
    def get_accel_lsb(self,Inputs,MassProperties):
        '''
        Returns:
            new_accel_lsb  (np.array(3)) = New accelerometer measurment 
            sensor_vecotor (np.array(6)) = New udpdated sensor vector
        Inputs:
            State          (Class)       = Current absolute state (see Kinematics.py)
        '''
        # Generate a the current acceleration plus accelerometer noise
        accel_variance_mps2     = 5
        new_accel_mps2          = add_noise(Inputs.forces_n/MassProperties.mass_kg,accel_variance_mps2)

        # Convert acceleration in m/s^2 to LSB (format the accelerometer actual outputs)    
        
        self.accel_lsb          = new_accel_mps2 * mps2_2_lsb
        self.sensor_vector[0:3] = self.accel_lsb

    def get_gyro_lsb(self,State,time):
        '''
        Returns:
            new_gyro_lsb   (np.array(3)) = New gyro measumrnets [theta,phi,psi]
            sensor_vecotor (np.array(6)) = New udpdated sensor vector
        Inputs:
            State          (Class)       = Current absolute state (see Kinematics.py)
        '''
        gyro_variance_rad       = 0.1
        gyro_bias_rad           = 2
        gyro_drift_radps        = 0.01

        # Calculate a new gyro measurment (+ bias + current_drift) and some noise
        new_gyro_radps          = State.w_radps
        self.sensor_vector[3]   = add_noise(new_gyro_radps[0]+ gyro_bias_rad + (gyro_drift_radps*time),gyro_variance_rad) * radps_2_lsb
        self.sensor_vector[4]   = add_noise(new_gyro_radps[1]+ gyro_bias_rad + (gyro_drift_radps*time),gyro_variance_rad) * radps_2_lsb
        self.sensor_vector[5]   = add_noise(new_gyro_radps[2]+ gyro_bias_rad + (gyro_drift_radps*time),gyro_variance_rad) * radps_2_lsb
    
