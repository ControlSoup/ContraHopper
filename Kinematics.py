# A set of functions designed to simulate the dynamics of a 6dof rigid body

import numpy as np
import strapdown


def state_derivative(state_vector, input_vector, g_mps2, vehicle_mass_kg, i_tensor_cg):
    [position_m, velocity_mps, attitude_Cb2i_dcm,
     w_radps] = state_vector  # the current Position (3x1), Velocity (3x1), Attitude from body to inertial frame (3x3) and Angular Rates (3x1)

    [force, moments] = input_vector  # Force and Moments in the body and moment about cg

    # Kinematics Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics

    # Resulting acceleration from the forcing acting on the body
    acceleration_mps2 = force / vehicle_mass_kg

    # Resulting angular acceleration due to the moments acting on the body
    wdot_radps2 = np.matmul(np.linalg.inv(i_tensor_cg),
                            (moments - (np.cross(w_radps,
                                                 (np.matmul(i_tensor_cg, w_radps))))))

    # Converts the current angular rates to a DCM rate
    attitude_Cb2idot_dcm = strapdown.rates2dcm_dot(w_radps)

    # Inputs: current state, forces and moments, gravity, vehicle mass, inertia tensor about the cg
    # Outptus: The derivative of the state based on 6dof rigid body kinematics
    return np.array(
        [velocity_mps, acceleration_mps2, attitude_Cb2idot_dcm, wdot_radps2])


# 4th Order Runge Kutta Calculation
def rk4(desired_integration_array, dt):
    # Inputs: an array that you want to integrate, and the desired change in time
    # Returns: the integral of the desired array
    # Calculate slope estimates
    K1 = desired_integration_array
    K2 = desired_integration_array + K1 * dt / 2
    K3 = desired_integration_array + K2 * dt / 2
    K4 = desired_integration_array + K3 * dt
    result = desired_integration_array + (1 / 6 * (K1 + 2 * K2 + 2 * K3 + K4) * dt)
    return np.array(result)


def new_state(state_vector, input_vector, dt, g_mps2=9.8055, vehicle_mass_kg=1, i_tensor_cg=None):
    if i_tensor_cg is None:
        i_tensor_cg = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    # Calculates the derivative of the current state
    state_dot = state_derivative(state_vector, input_vector, g_mps2, vehicle_mass_kg, i_tensor_cg)

    # Integrates the calculated derivative to find the next state
    next_state = rk4(state_dot, dt)
    return next_state
