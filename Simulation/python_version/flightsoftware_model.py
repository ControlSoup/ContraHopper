import numpy as np
import pathlib
import kinematics
from ctypes import *

"""
===========================
Classess
===========================
"""

class controller_3dof:
    def __init__(self,
                 Kp         = np.array([0,0,0.]),
                 Ki         = np.array([0,0,0.]),
                 Kd         = np.array([0,0,0.]),
                 pid_I      = np.array([0,0,0.]),
                 last_error = np.array([0,0,0.])):
        self.Kp         = np.array(Kp)
        self.Ki         = np.array(Ki)
        self.Kd         = np.array(Kd)
        self.pid_I      = np.array(pid_I)
        self.last_error = np.array(last_error)

"""
===========================
Functions
===========================
"""
def control_model(EstimatedState,TargetState,PositionController,dt):
    '''
    Returns:
        control_output_vector (np.array(6)) = Forces and moments disired by the controller
    '''
    # Position controller 
    hover_N = 98.1 # How much thrust is requried for a hover

    position_error = TargetState.position_m - EstimatedState.position_m
    postiion_pid_result = np.zeros(3)
    position_P          = np.zeros(3)
    position_D          = np.zeros(3)
    error_dot           = position_error- PositionController.last_error

    for i in range(len(position_error)):
        if (PositionController.pid_I[i] < 20 and PositionController.pid_I[i] > -20):
            PositionController.pid_I[i] += position_error[i] * PositionController.Ki[i] * dt
        else: 
            PositionController.pid_I[i] = 0

        position_P[i] = position_error[i] * PositionController.Kp[i]
        position_D[i] = error_dot[i] * PositionController.Kd[i] /dt

    PositionController.last_error = position_error
    postiion_pid_result = position_P + PositionController.pid_I + position_D

    # Add hover constant to z_position
    postiion_pid_result[2] += hover_N
    return np.array([postiion_pid_result,[0,0,0]]).flatten()


"""
===========================
Sim States
===========================
"""

TargetState = kinematics.State(position_m = [3,3,2.])

PositionController = controller_3dof(Kp = [10,10,10],Ki = [0.01,0.01,0.01], Kd = [3,3,3])
