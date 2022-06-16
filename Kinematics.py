# A set of functions designed to simulate the dynamics of a 6dof rigid body

import numpy as np
import strapdown


def state2state_matrix(State):
    return np.array([State.position_m,
            State.velocity_mps,
            State.attitude_Cb2i_dcm[0],
            State.attitude_Cb2i_dcm[1],
            State.attitude_Cb2i_dcm[2],
            State.w_radps])


def state_matrix2state(state_vector, State):
    State.position_m = np.array(state_vector[0])
    State.velocity_mps = np.array(state_vector[1])
    State.attitude_Cb2i_dcm = np.array([state_vector[2],
                               state_vector[3],
                               state_vector[4]])
    State.w_radps = np.array(state_vector[5])


def get_state_derivative(state_matrix, Inputs, g_mps2, MassProperties):
    """
    Inputs: State in matrix form, Forces and Moments about cg, Gravity , Vehicle Mass and Moment of Inerta Tensor at cg
    Primary Function: 6dof kinematics
    Outputs: Derivative of the current state based on 6dof kinematics as a matrix for integration
    """

    mass_kg = MassProperties.mass_kg  # Current mass of the vehicle
    i_tensor_cg = MassProperties.i_tensor_cg  # Current moment of inertia tensor at cg

    velocity_mps = state_matrix[1]  # Current velocity in meters per second
    attitude_Cb2i_dcm = [state_matrix[2],
                         state_matrix[3],
                         state_matrix[4]]  # Current attitude represented as a DCM from body to inertial frame
    w_radps = state_matrix[5]  # Current angular rate in radians per second

    forces_n = Inputs.forces_n  # Forces acting about cg
    moments_nm = Inputs.moments_nm  # Moments acting about cg

    # Kinematics Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics

    # Resulting acceleration from the forcing acting on the body
    acceleration_mps2 = forces_n / mass_kg
    acceleration_mps2[2] = acceleration_mps2[2] - g_mps2

    # Resulting angular acceleration due to the moments acting on the body
    wdot_radps2 = np.matmul(np.linalg.inv(i_tensor_cg),
                            (moments_nm - (np.cross(w_radps,
                                                    (np.matmul(i_tensor_cg, w_radps))))))

    # Converts the current angular rates to a DCM rate
    attitudedot_Cb2i_dcm = strapdown.rates2dcm(attitude_Cb2i_dcm, w_radps)

    return np.array([velocity_mps, acceleration_mps2, attitudedot_Cb2i_dcm[0], attitudedot_Cb2i_dcm[1], attitudedot_Cb2i_dcm[2],
            wdot_radps2])


def rk4(state_matrix, Inputs, g_mps2, MassProperties, dt):
    """
    RK4 Integration Algorithm
    """
    k1 = get_state_derivative(state_matrix, Inputs, g_mps2, MassProperties)
    k2 = get_state_derivative(state_matrix + (k1 * dt / 2), Inputs, g_mps2, MassProperties)
    k3 = get_state_derivative(state_matrix + (k2 * dt / 2), Inputs, g_mps2, MassProperties)
    k4 = get_state_derivative(state_matrix + (k3 * dt), Inputs, g_mps2, MassProperties)

    return state_matrix + np.array((k1 + 2 * k2 + 2 * k3 + k4) * dt / 6)




def get_new_state(State, Inputs, g_mps2, MassProperties, dt):

    """
    Inputs: state, forces and moments, dt, gravity, mass and moment of inertia tensor at cg
    Primary Function: Rk4 integration of the state derivative
    Outputs: the new state based on these parameters
    """

    state_matrix = state2state_matrix(State)

    next_state_matrix = rk4(state_matrix, Inputs, g_mps2, MassProperties, dt)

    state_matrix2state(next_state_matrix, State)
    State.attitude_Cb2i_dcm = strapdown.orthonormalize(State.attitude_Cb2i_dcm)

    return None
