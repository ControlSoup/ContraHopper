# A set of functions designed to simulate the dynamics of a 6dof rigid body

import numpy as np
import strapdown

"""
===========================
Classes
===========================
"""

class State:
    '''
    Stores state in vector and variable form
    '''
    def __init__(self,position_m=np.zeros(3),velocity_mps=np.zeros(3),
                 Cb2i_dcm=np.identity(3),w_radps=np.zeros(3)):
                 self.position_m      = position_m
                 self.velocity_mps    = velocity_mps
                 self.Cb2i_dcm        = Cb2i_dcm
                 self.w_radps         = w_radps
                 self.state_matrix    = np.array([position_m,velocity_mps,
                                         Cb2i_dcm[0],
                                         Cb2i_dcm[1],
                                         Cb2i_dcm[2],
                                         w_radps])

    def update_from_state_matrix(self,state_matrix):
        '''
        Overview:
            Updates current class properties
        Inputs:
            state_matrix (np.array[3][6]) = Current state derivative in vector form
        '''
        self.state_matrix = state_matrix
        self.position_m   = state_matrix[0]
        self.velocity_mps = state_matrix[1]
        self.Cb2i_dcm     =[state_matrix[2],
                            state_matrix[3],
                            state_matrix[4]]
        self.w_radps      = state_matrix[5]

class Inputs:
    '''
    Stores forces and moments as a vector and variable form

    Properties:
        forces_n     (np.array[3])    = Forces acting on the body
        moments_nm   (np.array[3])    = Moments acting on the body
        input_matrix (np.array[2][3]) = Forces and moments in vector form
    '''
    def __init__(self,forces_n=np.zeros(3),moments_nm=np.zeros(3)):
        self.forces_n         =  forces_n
        self.moments_nm       =  moments_nm
        self.input_matrix     =  np.array([forces_n,moments_nm])

    def update_from_input_matrix(self,input_matrix):
        '''
        Overview:
            Updates current class properties
        Inputs:
            input_matrix (np.array) = Current inputs in vector form
        '''
        self.input_matrix     =  input_matrix
        self.forces_n         =  input_matrix[0]
        self.moments_nm       =  input_matrix[1]

class MassProperties:
    '''
    Contains all mass properties properties
    '''
    def __init__(self,mass_kg = -10.0,i_tensor_cg = np.identity(3)):
                 self.mass_kg = mass_kg
                 self.i_tensor_cg = i_tensor_cg
 
"""
===========================
State Derivative
===========================
"""

def get_state_derivative(state_matrix,Inputs,MassProperties):
    """
    Returns:
        statedot_matrix (np.array[3][6]) = Current state derivative in vector form
    Inputs: 
        state_matrix    (np.array[3][6]) = Current state in vector form
        Inputs          (Class)          = Forces and moments acting on the rigid body
        MassProperties  (Class)          = Mass and inertia tensor of the rigid body
    Notes:
        Kinematics Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics
    """

    # Parse Inputs 
    velocity_mps = state_matrix[1]
    Cb2i_dcm     =[state_matrix[2],
                   state_matrix[3],
                   state_matrix[4]]
    w_radps      = state_matrix[5]
    forces_n     = Inputs.forces_n
    moments_nm   = Inputs.moments_nm
    mass_kg      = MassProperties.mass_kg
    i_tensor_cg  = MassProperties.i_tensor_cg

    # Calculate forces corected in the body frame (eq 3.1-21 and 3.1-12 Strapdown Analytics)
    forces_n = np.matmul(Cb2i_dcm,forces_n)

    # Calculate acceleration from the forces acting on the body
    # a = F/m
    acceleration_mps2 = forces_n / mass_kg

    # Calculate angular acceleration due to the moments acting on the body
    # w = I^-1(M-(w ×(Iw))
    wdot_radps2 = np.matmul(np.linalg.inv(i_tensor_cg),
                            (moments_nm - (np.cross(w_radps,(np.matmul(i_tensor_cg, w_radps))))))

    # Converts the current angular rates to a DCM rate
    Cb2idot_dcm = strapdown.rates2dcm(Cb2i_dcm, w_radps)
    # Orthonormalize 
    Cb2idot_dcm = strapdown.orthonormalize(Cb2idot_dcm) 
    return np.array([velocity_mps, acceleration_mps2, 
                     Cb2idot_dcm[0], Cb2idot_dcm[1], 
                     Cb2idot_dcm[2], wdot_radps2])

"""
===========================
Integration
===========================
"""

def rk4(state_matrix, Inputs, MassProperties, dt):
    """
    Returns:
        new_state_matrix (np.array[3][6]) = Rk4 integration of the current state
    Inputs: 
        state_matrix     (np.array[3][6]) = Current state in vector form
        Inputs           (Class)        = Forces and moments acting on the rigid body
        MassProperties   (Class)        = Mass and inertia tensor of the rigid body
        dt               (float)        = Time between integration steps 
    Notes:
        Source : https://medium.com/geekculture/runge-kutta-numerical-integration-of-ordinary-differential-equations-in-python-9c8ab7fb279c
    """
    
    k1 = get_state_derivative(state_matrix, Inputs, MassProperties)
    k2 = get_state_derivative(state_matrix + (k1 * dt / 2), Inputs, MassProperties)
    k3 = get_state_derivative(state_matrix + (k2 * dt / 2), Inputs, MassProperties)
    k4 = get_state_derivative(state_matrix + (k3 * dt), Inputs, MassProperties)

    return state_matrix + np.array((k1 + 2 * k2 + 2 * k3 + k4) * dt / 6)

    
