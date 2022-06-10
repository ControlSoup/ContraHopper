# Change Log
# Added Pitch Controller, implemented noise to the control loop and tuned some of the pid loops
# Added a imu model start
# Any amount of noise or setpoint error that isint perfect fucks everthing up


import numpy as np
import matplotlib.pyplot as plt


# 4th Order Runge Kutta Calculation
def RK4(x, u, dt):
    # Inputs: x[k], u[k], dt (time step, seconds)
    # Returns: x[k+1]
    # Calculate slope estimates
    K1 = stateDerivative(x, u)
    K2 = stateDerivative(x + K1 * dt / 2, u)
    K3 = stateDerivative(x + K2 * dt / 2, u)
    K4 = stateDerivative(x + K3 * dt, u)
    # Calculate x[k+1] estimate using combination of slope estimates
    x_next = x + (1 / 6 * (K1 + 2 * K2 + 2 * K3 + K4) * dt)
    return np.array(x_next)


def stateDerivative(state_vector, input_vector, g=9.8055):
    # Vehicle Properties
    mass_kg = 1
    i_tensor =[[1,0,0],[0,1,0],[0,0,1]] #[[0.4004, 0, 0], [0, 0.7391, 0], [0, 0, 0.4004]]

    [x, y, z, x_dot, y_dot, z_dot, pitch, yaw, roll, pitch_dot, yaw_dot, roll_dot] = state_vector
    [fx, fy, fz, mx, my, mz] = input_vector  # Force and Moments in the body and moment about cg
    force = np.array([fx, fy, fz])  # Force acting on the body in newtons
    moments = np.array([mx, my, mz])  # Force acting on the body in newtons/meter
    attitude_euler_dot = np.array(
        [pitch_dot, yaw_dot, roll_dot])  # Attiude rate in radians and euler coordinates (Pitch -> Yaw -> Roll)

    # Source: https://en.m.wikipedia.org/wiki/Rigid_body_dynamics

    # Resulting acceleration from the forcing acting on the body
    acc = force / mass_kg

    # Resulting angular acceleration due to the moments acting on the body
    alpha = np.matmul(np.linalg.inv(i_tensor),
                      (moments - (np.cross(attitude_euler_dot,
                                           (np.matmul(i_tensor, attitude_euler_dot))))))
    x_ddot = acc[0]
    y_ddot = acc[1]
    z_ddot = acc[2]
    pitch_ddot = alpha[0]
    yaw_ddot = alpha[1]
    roll_ddot = alpha[2]
    return np.array(
        [x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, pitch_dot, roll_dot, yaw_dot, pitch_ddot, roll_ddot, yaw_ddot])

# Magic Numbers
in2m = 0.0254
kg2n = 9.8055

# Sim Options
sim_frequency = 250
start_time = 0
end_time = 120
dt = 1 / sim_frequency
sim_t = np.arange(start_time, end_time, dt)

isPlotting = False
is3d = True
# Initial State
state_itt = np.zeros((len(sim_t), 14))
state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
force = [0,0,0,0,0,0]
for i in range(0, len(sim_t)):
    # if sim_t[i]-control_timer >= control_dt:
    if i < 2:
        force[5] = 0.1
    elif i >= 2:
        force[5] = -0.1
    elif i > 4:
        force[5] = 0
    if i > 0:
        state = state_itt[i - 1][:]

    state_itt[i][0:12] = RK4(state[0:12], force, dt)
    state_itt[i][13] = force[5]

if isPlotting:
    plt.subplots(4, 1)

    plt.subplot(4, 1, 1)
    plt.plot(sim_t, state_itt[:, 0], 'r--', label="Position X")
    plt.plot(sim_t, state_itt[:, 1], 'k--', label="Position Y")
    plt.plot(sim_t, state_itt[:, 2], 'b--', label="Position Z")
    plt.legend()
    plt.xlabel('Time(s)')
    plt.ylabel('Position (m)')

    plt.subplot(4, 1, 2)
    plt.plot(sim_t, state_itt[:, 3], 'r--', label="Velocity X")
    plt.plot(sim_t, state_itt[:, 4], 'k--', label="Velocity Y")
    plt.plot(sim_t, state_itt[:, 5], 'b--', label="Velocity Z")
    plt.legend()
    plt.xlabel('Time(s)')
    plt.ylabel('Velocity (m/s)')

    plt.subplot(4, 1, 3)
    plt.plot(sim_t, state_itt[:, 6], 'r--', label="Pitch")
    plt.plot(sim_t, state_itt[:, 7], 'k--', label="Yaw")
    plt.plot(sim_t, state_itt[:, 8], 'b--', label="Roll")
    plt.legend()
    plt.xlabel('Time(s)')
    plt.ylabel('Attitude (rad)')

    plt.subplot(4, 1, 4)
    plt.plot(sim_t, state_itt[:, 9], 'r--', label="Pitch Rate")
    plt.plot(sim_t, state_itt[:, 10], 'k--', label="Yaw Rate")
    plt.plot(sim_t, state_itt[:, 11], 'b--', label="Roll Rate")
    plt.legend()
    plt.xlabel('Time(s)')
    plt.ylabel('Attitude Rate (rad/s)')

    plt.subplots_adjust(left=0.1,
                        bottom=0.1,
                        right=0.9,
                        top=0.9,
                        wspace=0.4,
                        hspace=0.4)

    plt.show()

    plt.plot(sim_t, sim, 'r--', label="New Pendulum")
    plt.legend()
    plt.xlabel('Fill_height(m)')
    plt.ylabel('Pendulum Mass(kg)')
    plt.show()

