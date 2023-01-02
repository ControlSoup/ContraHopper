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
    Overview:
        Stores a state in vector and variable format, using update functions
    Properties:
        position_m   (np.array(3))    = Position of the rigid body (x,y,z)
        velocity_mps (np.array(3))    = velocity of the rigid body (x,y,z)
        Cb2i_dcm     (np.array(3)(3)) = Attitude Dcm from body to intertia 
        w_radps      (np.arrau(3))    = Attitude rate about cg of the rigid body (theta,phi,psi)
    '''
    def __init__(self,position_m=np.zeros(3),velocity_mps=np.zeros(3),
                 Cb2i_dcm=np.identity(3),w_radps=np.zeros(3)):

        self.position_m   = np.array(position_m)
        self.velocity_mps = np.array(velocity_mps)
        self.Cb2i_dcm     = np.array(Cb2i_dcm)
        self.w_radps      = np.array(w_radps)
        self.state_vector = np.array([position_m,
                                      velocity_mps,
                                      Cb2i_dcm[0],
                                      Cb2i_dcm[1],
                                      Cb2i_dcm[2],
                                      w_radps]).flatten()
                    
                
    def update_from_state_vector(self,state_vector):
        '''
        Overview:
            Updates current class properties from state_vector form
        Inputs:
            state_vector (np.array[1x18]) = Current state in vector form
        '''
        self.state_vector = np.array(state_vector)
        self.position_m   = np.array(self.state_vector[0:3])
        self.velocity_mps = np.array(self.state_vector[3:6])
        self.Cb2i_dcm     = np.array([self.state_vector[6:9],
                                      self.state_vector[9:12],
                                      self.state_vector[12:15]])
        self.w_radps      = np.array(self.state_vector[15:18])

    def update_from_properties(self,position_m,velocity_mps,
                 Cb2i_dcm,w_radps):

        self.position_m   = np.array(position_m)
        self.velocity_mps = np.array(velocity_mps)
        self.Cb2i_dcm     = np.array(Cb2i_dcm)
        self.w_radps      = np.array(w_radps)
        self.state_vector = np.array([position_m,
                                      velocity_mps,
                                      Cb2i_dcm[0],
                                      Cb2i_dcm[1],
                                      Cb2i_dcm[2],
                                      w_radps]).flatten()

class Inputs:
    '''
    Overview:
        Stores inputs in vector and variable format, using update functions

    Properties:
        forces_n     (np.array(3)) = Forces acting on the body
        moments_nm   (np.array(3)) = Moments acting on the body
        input_vector (np.array(6)) = Forces and moments in vector form
    '''
    def __init__(self,forces_n=np.zeros(3),moments_nm=np.zeros(3)):
        self.forces_n         =  forces_n
        self.moments_nm       =  moments_nm
        self.input_vector     =  np.array([forces_n,moments_nm]).flatten()

    def update_from_input_vector(self,input_vector):
        '''
        Overview:
            Updates current class properties
        Inputs:
            input_vector (np.array(6)) = Current inputs in vector form
        '''
        self.input_vector     =  np.array(input_vector)
        self.forces_n         =  np.array(input_vector[0:3])
        self.moments_nm       =  np.array(input_vector[3:6])

    def update_from_properties(self,forces_n,moments_nm):
        '''
        Overview:
            Updates current class properties
        Inputs:
            forces_n   = Current forces acting on the body in newtons (x,y,z)
            moments_nm = Moments acting about cg (theta,psi,phi) in newtons per meter
        '''
        self.forces_n         =  np.array(forces_n)
        self.moments_nm       =  np.array(moments_nm)
        self.input_vector     =  np.array([forces_n,moments_nm]).flatten()
class MassProperties:
    '''
    Contains all mass properties
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
    Returns:
        statedot_matrix (np.array(18)) = Current state derivative in vector form
    Inputs: 
        state_vector    (np.array(18)) = Current state in vector form
        Inputs          (Class)        = Forces and moments acting on the rigid body
        MassProperties  (Class)        = Mass and inertia tensor of the rigid body
    Notes:
        Kinematics Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics
    """

    # Parse Inputs 
    velocity_mps = np.array(state_vector[3:6])
    Cb2i_dcm     = np.array([state_vector[6:9],
                             state_vector[9:12],
                             state_vector[12:15]])
    w_radps      = np.array(state_vector[15:18])
    forces_n     = Inputs.forces_n
    moments_nm   = Inputs.moments_nm
    mass_kg      = MassProperties.mass_kg
    i_tensor_cg  = MassProperties.i_tensor_cg

    # Calculate velocity corrected in the body frame (eq 3.1-21 and 3.1-12 Strapdown Analytics)
    velocity_mps      = np.matmul(Cb2i_dcm,velocity_mps)

    # Calculate forces corrected in the body frame 
    forces_n          = np.matmul(Cb2i_dcm,forces_n)

    # Calculate acceleration from the forces acting on the body
    # a = F/m
    acceleration_mps2 = forces_n / mass_kg

    # Calculate angular acceleration due to the moments acting on the body
    # w = I^-1(M-(w ×(Iw))
    wdot_radps2       = np.matmul(np.linalg.inv(i_tensor_cg),
                                  (moments_nm - (np.cross(w_radps,(np.matmul(i_tensor_cg, w_radps))))))

    # Converts the current angular rates to a DCM rate
    Cb2idot_dcm       = strapdown.rates2dcm(Cb2i_dcm, w_radps)

    # Orthonormalize 
    #Cb2idot_dcm       = strapdown.orthonormalize(Cb2idot_dcm) 
    
    return np.array([velocity_mps, 
                     acceleration_mps2, 
                     Cb2idot_dcm[0], 
                     Cb2idot_dcm[1],
                     Cb2idot_dcm[2],
                     wdot_radps2]).flatten()

"""
===========================
Integration
===========================
"""

def rk4(state_vector, Inputs, MassProperties, dt):
    """
    Returns:
        new_state_vector (np.array(18)) = Rk4 integration of the current state
    Inputs: 
        state_vector     (np.array(18)) = Current state in vector form
        Inputs           (Class)        = Forces and moments acting on the rigid body
        MassProperties   (Class)        = Mass and inertia tensor of the rigid body
        dt               (float)        = Time between integration steps 
    Notes:
        Source : https://medium.com/geekculture/runge-kutta-numerical-integration-of-ordinary-differential-equations-in-python-9c8ab7fb279c
    """
    
    k1 = get_state_derivative(state_vector, Inputs, MassProperties)
    k2 = get_state_derivative(state_vector + (k1 * dt / 2), Inputs, MassProperties)
    k3 = get_state_derivative(state_vector + (k2 * dt / 2), Inputs, MassProperties)
    k4 = get_state_derivative(state_vector + (k3 * dt), Inputs, MassProperties)

    return state_vector + np.array((k1 + 2 * k2 + 2 * k3 + k4) * dt / 6)



    
# Test State
# TestState = State([9,9,9],[9,9,9],np.identity(3),[9,9,9])
# print(TestState.position_m)
# print(TestState.state_vector)
# TestState.update_from_state_vector([1.,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18])
# print(TestState.position_m)
# print(TestState.state_vector)
# TestState.update_from_properties(position_m=[1.,1,1],
#                                  velocity_mps=[2.,2,2],
#                                  Cb2i_dcm=[[1.,1,1],
#                                            [6,7,9],
#                                            [3,3,7]],
#                                  w_radps=[1,1,1])
# print(TestState.position_m)
# print(TestState.state_vector)