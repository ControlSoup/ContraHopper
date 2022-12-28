import numpy as np
from numpy.random import randn
"""
===========================
Supporting function
===========================
"""
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
    def __init__(self,accel_raw = np.zeros(3),sensor_vector = np.zeros(3)):
                 self.accel_raw       = accel_raw
                 self.sensor_vector   = np.array([self.accel_raw])
    
    def new_accel_raw(self,State):
        '''
        Returns:
            new_accel_raw  (np.array[3]) = new accel_rawerometer measurment 
            sensor_vecotor (np.array[0][3])
        Inputs:
            State          (Class)       = Current absolute state (see Kinematics.py)
        '''
        # Generate a the current acceleration plus accelerometer noise
        accel_variance = 6
        new_accel_raw  = add_noise(State.acceleration_mps2,accel_variance)

        # Coonvert acceleration in m/s^2 to LSB (format the accelerometer actual outputs)    
        accel2lsb      = 1046 
        self.accel_raw = new_accel_raw * accel2lsb