# A set of functions designed to simulate the dynamics of a 6dof rigid body

import numpy as np
import strapdown

"""
===========================
Support Functions
===========================
"""

class State:
    '''
    Stores state in variable and vector format
    '''
    def __init__(self,position_m=np.zeros(3),velocity_mps=np.zeros(3),
                 Cb2i_dcm=np.identity(3),w_radps=np.zeros(3)):
        self.position_m      = position_m
        self.velocity_mps    = velocity_mps
        self.Cb2i_dcm        = Cb2i_dcm
        self.w_radps         = w_radps
        self.state_vector    = np.array([self.position_m,
                                         self.velocity_mps,
                                         self.Cb2i_dcm[0],
                                         self.Cb2i_dcm[1],
                                         self.Cb2i_dcm[2],
                                         self.w_radps])

    def update_from_state_vector(self,state_vector):
        self.state_vector = state_vector
        self.position_m   = state_vector[0]
        self.velocity_mps = state_vector[1]
        self.Cb2i_dcm     =[state_vector[2],
                            state_vector[3],
                            state_vector[4]]
        self.w_radps      = state_vector[5]

   


class Inputs:
    '''
    Stores forces and moments as a vector and variable format
    '''
    def __init__(self,forces_n=np.zeros(3),moments_nm=np.zeros(3)):
        self.forces_n         =  forces_n
        self.moments_nm       =  moments_nm
        self.input_vector     =  np.array([forces_n,moments_nm])

    def update_from_input_vector(self,input_vector):
        self.input_vector     =  input_vector
        self.forces_n         =  input_vector[0]
        self.moments_nm       =  input_vector[1]

class MassProperties:
    '''
    Contains all vehicles properties
    '''
    def __init__(self,mass_kg = -10.0,i_tensor_cg = np.identity(3)):
        self.mass_kg = mass_kg
        self.i_tensor_cg = i_tensor_cg
 
"""
===========================
State Derivative
===========================
"""

def get_state_derivative(state_vector,Inputs,MassProperties):
    """
    Inputs: State in matrix form, Forces and Moments about cg, Gravity , Vehicle Mass and Moment of Inerta Tensor at cg
    Outputs: Derivative of the current state based on 6dof kinematics as a vector for integration
    """

    # Parse Inputs 
    position_m   = state_vector[0]
    velocity_mps = state_vector[1]
    Cb2i_dcm     =[state_vector[2],
                   state_vector[3],
                   state_vector[4]]
    w_radps      = state_vector[5]
    forces_n     = Inputs.forces_n
    moments_nm   = Inputs.moments_nm
    mass_kg      = MassProperties.mass_kg
    i_tensor_cg  = MassProperties.i_tensor_cg

    Inputs.forces_n = np.matmul(Cb2i_dcm,forces_n)  # Forces acting in the body frame
    # (corrected from inertial frame) (eq 3.1-21 and 3.1-12 from strapdown)

    # Kinematics Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics
    # Resulting acceleration from the forcing acting on the body
    acceleration_mps2 = forces_n / mass_kg

    # Resulting angular acceleration due to the moments acting on the body
    wdot_radps2 = np.matmul(np.linalg.inv(i_tensor_cg),
                            (moments_nm - (np.cross(w_radps,(np.matmul(i_tensor_cg, w_radps))))))

    # Converts the current angular rates to a DCM rate
    Cb2idot_dcm = strapdown.rates2dcm(Cb2i_dcm, w_radps)

    return np.array(
        [velocity_mps, acceleration_mps2, Cb2idot_dcm[0], Cb2idot_dcm[1], Cb2idot_dcm[2],
         wdot_radps2])


"""
===========================
Integration
===========================
"""


def rk4(state_vector, Inputs, MassProperties, dt):
    """
    RK4 Integration Algorithm
    """
    k1 = get_state_derivative(state_vector, Inputs, MassProperties)
    k2 = get_state_derivative(state_vector + (k1 * dt / 2), Inputs, MassProperties)
    k3 = get_state_derivative(state_vector + (k2 * dt / 2), Inputs, MassProperties)
    k4 = get_state_derivative(state_vector + (k3 * dt), Inputs, MassProperties)

    return state_vector + np.array((k1 + 2 * k2 + 2 * k3 + k4) * dt / 6)

    
