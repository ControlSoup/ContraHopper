# A set of functions designed to simulate the dynamics of a 6dof rigid body

import numpy as np
import strapdown


class Resulting_state():
    def __init__(self, velocity_mps, acceleration_mps2, attitudedot_Cb2i_dcm, wdot_radps2):
        self.velocity_mps = np.array(velocity_mps)
        self.acceleration_mps2 = np.array(acceleration_mps2)
        self.attitudedot_Cb2i_dcm = np.array(attitudedot_Cb2i_dcm)
        self.wdot_radps2 = np.array(wdot_radps2)
class Rk4_class():
    def __init__(self, position_m, velocity_mps, attitude_Cb2i_dcm, w_radps):
        self.position_m = np.array(position_m)
        self.velocity_mps = np.array(velocity_mps)
        self.attitude_Cb2i_dcm = np.array(attitude_Cb2i_dcm)
        self.w_radps = np.array(w_radps)


def state_derivative(State, Inputs, g_mps2, Mass_Properties, Resulting_State):
    '''
    Inputs: State, Forces and Moments, Gravity , Vehicle Mass and Moment of Inerta Tensor at cg
    Primary Function: 6dof kinematics
    Outputs: Derivative of the current state based on 6dof kinematics
    '''
    mass_kg = Mass_Properties.mass_kg  # Current mass of the vehicle
    i_tensor_cg = Mass_Properties.i_tensor_cg  # Current moment of inertia tensor at cg

    position_m = State.position_m  # Current position in meters
    velocity_mps = State.velocity_mps  # Current velocity in meters per second
    attitude_Cb2i_dcm = State.attitude_Cb2i_dcm  # Current attitude represented as a DCM from body to inertial frame
    w_radps = State.w_radps  # Current angular rate in radians per second

    forces_n = Inputs.forces_n  # Forces acting about cg
    moments_nm = Inputs.moments_nm  # Moments acting about cg

    # Kinematics Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics

    # Resulting acceleration from the forcing acting on the body
    acceleration_mps2 = forces_n / mass_kg
    acceleration_mps2[2] = acceleration_mps2[2]-g_mps2

    # Resulting angular acceleration due to the moments acting on the body
    wdot_radps2 = np.matmul(np.linalg.inv(i_tensor_cg),
                            (moments_nm - (np.cross(w_radps,
                                                     (np.matmul(i_tensor_cg, w_radps))))))

    # Converts the current angular rates to a DCM rate
    attitudedot_Cb2i_dcm = strapdown.rates2dcm(attitude_Cb2i_dcm,w_radps)
    Resulting_State.velocity_mps = velocity_mps
    Resulting_State.acceleration_mps2 = acceleration_mps2
    Resulting_State.attitudedot_Cb2i_dcm = attitudedot_Cb2i_dcm    # Cdot_B2I
    Resulting_State.wdot_radps2 = wdot_radps2


    return None



def new_state(State, Inputs, dt, g_mps2, Mass_Properties):
    '''
    Inputs: state, forces and moments, dt, gravity, mass and moment of inertia tensor at cg
    Primary Function: Rk4 integration of the state derivative
    Outpus: the new state based on these parameters
    '''

    # 4th Order Runge Kutta Calculation
    # K1
    k1 = Resulting_state(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]),
                         np.array([0, 0, 0]))

    state_derivative(State, Inputs, g_mps2, Mass_Properties, k1)

    # K2
    k2_dot = Rk4_class(np.array(State.position_m + k1.velocity_mps * dt / 2),
                       np.array(State.velocity_mps + k1.acceleration_mps2 * dt / 2),
                       np.array(State.attitude_Cb2i_dcm + k1.attitudedot_Cb2i_dcm * dt / 2),
                       np.array(State.w_radps + k1.wdot_radps2 * dt / 2))
    k2 = Resulting_state(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                         np.array([0, 0, 0]))
    state_derivative(k2_dot, Inputs, g_mps2, Mass_Properties, k2)
    #K3
    k3_dot = Rk4_class(np.array(State.position_m + k2.velocity_mps * dt / 2),
                       np.array(State.velocity_mps + k2.acceleration_mps2 * dt / 2),
                       np.array(State.attitude_Cb2i_dcm + k2.attitudedot_Cb2i_dcm * dt / 2),
                       np.array(State.w_radps + k2.wdot_radps2 * dt / 2))
    k3 = Resulting_state(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                         np.array([0, 0, 0]))
    state_derivative(k3_dot, Inputs, g_mps2, Mass_Properties, k3)

    #K4
    k4_dot = Rk4_class(np.array(State.position_m + k3.velocity_mps * dt),
                       np.array(State.velocity_mps + k3.acceleration_mps2 * dt),
                       np.array(State.attitude_Cb2i_dcm + k3.attitudedot_Cb2i_dcm * dt),
                       np.array(State.w_radps + k3.wdot_radps2 * dt))
    k4 = Resulting_state(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                         np.array([0, 0, 0]))
    state_derivative(k4_dot, Inputs, g_mps2, Mass_Properties, k4)

    #Integration Results
    rk4_result = Rk4_class(State.position_m + (1 / 6 * (k1.velocity_mps + 2 * k2.velocity_mps + 2 * k3.velocity_mps + k4.velocity_mps) * dt),
                           State.velocity_mps + (1 / 6 * (k1.acceleration_mps2 + 2 * k2.acceleration_mps2 + 2 * k3.acceleration_mps2 + k4.acceleration_mps2) * dt),
                           State.attitude_Cb2i_dcm + (1 / 6 * (k1.attitudedot_Cb2i_dcm + 2 * k2.attitudedot_Cb2i_dcm + 2 * k3.attitudedot_Cb2i_dcm + k4.attitudedot_Cb2i_dcm) * dt),
                           State.w_radps + (1 / 6 * (k1.wdot_radps2 + 2 * k2.wdot_radps2 + 2 * k3.wdot_radps2 + k4.wdot_radps2) * dt))


    #rk4_result.attitude_Cb2i_dcm = strapdown.orthonormalize(rk4_result.attitude_Cb2i_dcm)

    State.position_m = rk4_result.position_m
    State.velocity_mps = rk4_result.velocity_mps
    State.attitude_Cb2i_dcm = rk4_result.attitude_Cb2i_dcm
    State.w_radps = rk4_result.w_radps
    return None
