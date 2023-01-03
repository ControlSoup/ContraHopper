import numpy as np
import pathlib
import kinematics
from ctypes import *

TargetState = kinematics.State(position_m=[0,0,10],
                               velocity_mps=[0,0,0],
                               Cb2i_dcm=[[1,0,0],
                                         [0,1,0],
                                         [0,0,1]],
                               w_radps=[0,0,0])

# control function
c_functions = CDLL(str(pathlib.Path(__file__).parent.resolve())+"..\..\..\FlightSoftware\Simulator\\flight_software_sim.so")
c_functions.control.argtypes = [POINTER(c_double),POINTER(c_double),POINTER(c_double),c_double]
c_functions.control.restype  = POINTER(c_double)

pid_storage = np.array([0,0,0,0,                    # pos_x
                        0,0,0,0,                    # pos_y
                        1,0,0,0],dtype = c_double)  # pos_z

def control_sim(State,TargetState,Inputs,pid_storage,dt):
    # Convert state_vectors to c_double arrays 
    c_state_vector        = np.array(State.state_vector,dtype = c_double).ctypes.data_as(POINTER(c_double))
    c_target_state_vector = np.array(TargetState.state_vector,dtype = c_double).ctypes.data_as(POINTER(c_double))
    c_pid_storage         = pid_storage.ctypes.data_as(POINTER(c_double))
    control_output        = c_functions.control(c_state_vector,
                                                c_target_state_vector,
                                                c_pid_storage,dt) 

    # Parse control_ouput

    # Position X
    pid_storage[2] = control_output[6] 
    pid_storage[3] = control_output[7] 

    # Position Y
    pid_storage[6] = control_output[8] 
    pid_storage[7] = control_output[9] 

    # Position Z
    pid_storage[10] = control_output[10]
    pid_storage[11] = control_output[11]
    
    Inputs.update_from_properties([control_output[0],control_output[1],control_output[2]],
                                  [control_output[3],control_output[4],control_output[5]])

# Array passing to a C function Test (works)
# array = np.array([0,0,0,0,0,0],dtype=c_double)
# array = array.ctypes.data_as(POINTER(c_double))
# array_result = c_functions.flight_software_sim(array,axis_scale.0,1.0)
# array_result = np.array([array_result[0],array_result[1],array_result[2],
#                          array_result[3],array_result[4],array_result[5]])
# print(array_result)

c_state_vector        = np.array(TargetState.state_vector,dtype = c_double).ctypes.data_as(POINTER(c_double))
c_target_state_vector = np.array(TargetState.state_vector,dtype = c_double).ctypes.data_as(POINTER(c_double))
c_pid_storage         = pid_storage.ctypes.data_as(POINTER(c_double))
c_functions.control(c_state_vector,
                    c_target_state_vector,
                    c_pid_storage,0.05) 