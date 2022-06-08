
# Change Log
#Added Pitch Controller, implemented noise to the control loop and tuned some of the pid loops
#Added a imu model start
#Any amount of noise or setpoint error that isint perfect fucks everthing up

from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt


# 4th Order Runge Kutta Calculation
def RK4(x,u,dt):
    # Inputs: x[k], u[k], dt (time step, seconds)
    # Returns: x[k+1]
    # Calculate slope estimates
    K1 = stateDerivative(x, u)
    K2 = stateDerivative(x + K1 * dt / 2, u)
    K3 = stateDerivative(x + K2 * dt / 2, u)
    K4 = stateDerivative(x + K3 * dt, u)
    # Calculate x[k+1] estimate using combination of slope estimates
    x_next = x + (1/6 * (K1+ 2*K2 + 2*K3 + K4) * dt)
    return np.array(x_next)

def stateDerivative(state_vector,input_vector,g=9.8055):
    #Vehicle Properties
    mass_kg  = 100
    i_tensor = [[1,0,0],[0,1,0],[0,0,1]]

    [x,y,z,x_dot,y_dot,z_dot,pitch,yaw,roll,pitch_dot,yaw_dot,roll_dot] = state_vector
    [fx,fy,fz,mx,my,mz] = input_vector #Force in the body and moment about cg
    force = np.array([fx,fy,fz])
    moments = np.array([mx,my,mz])
    position = np.array([x,y,z])
    velocity = np.array([x_dot,y_dot,z_dot])
    attitude_euler = np.array([pitch,yaw,roll])
    attitude_euler_dot = np.array([pitch_dot,yaw_dot,roll_dot])

    acc = force/mass_kg
    alpha =

    x_ddot = acc[0]
    y_ddot = acc[1]
    z_ddot = acc[2]
    pitch_ddot = 0
    yaw_ddot = 0
    roll_ddot = 0
    return np.array([x_dot,y_dot,z_dot,x_ddot,y_ddot,z_ddot,pitch_dot,roll_dot,yaw_dot,pitch_ddot,roll_ddot,yaw_ddot])




# Magic Numbers
in2m = 0.0254
kg2n= 9.8055



# Sim Options
sim_frequency = 250
start_time = 0
end_time = 120
dt = 1/sim_frequency
sim_t = np.arange(start_time,end_time,dt)

#Inital State
state_itt = []
state = [0,0,0,0,0,0,0,0,0,0,0,0]
inputs = [1,0,0,1,0,0]

print(stateDerivative(state,inputs))

for i in range (0,len(sim_t)):
    break
    #to the center of the eye we go

    # if sim_t[i]-control_timer >= control_dt:
    # break

    #
    # state_itt[i][0:6] = RK4(np.array(state[0:12]),np.array([0,0,0,0,0,1]),dt)


# plt.subplots(5, 1)
#
# plt.subplot(5, 1, 1)
# plt.plot(sim_t, state_itt[:,11], 'r--', label="Error")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('Pitch Error')
#
# plt.subplot(5, 1, 2)
# plt.plot(sim_t, state_itt[:,8], 'r--', label="pitch_p")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('P Term')
#
# plt.subplot(5, 1, 3)
# plt.plot(sim_t, state_itt[:,9], 'r--', label="pitch_i")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('I Term')
#
# plt.subplot(5, 1, 4)
# plt.plot(sim_t, state_itt[:,10], 'r--', label="pitch_d")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('D Term')
#
# plt.subplot(5, 1, 5)
# plt.plot(sim_t, state_itt[:,12], 'r--', label="pitch_d")
# plt.xlabel('Time(s)')
# plt.ylabel('Pitch Desired')
#
# plt.show()
#
# plt.subplots(4, 1)
#
# plt.subplot(4, 1, 1)
# plt.plot(sim_t, state_itt[:,6], 'r--', label="m1_force")
# plt.plot(sim_t, state_itt[:,7], 'k--', label="m2_force")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('Thrust(N)')
#
# plt.subplot(4, 1, 2)
# plt.plot(sim_t, state_itt[:,0], 'r--', label="x_pos")
# plt.plot(sim_t, state_itt[:,1], 'k--', label="y_pos")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('Pos(m)')
#
# plt.subplot(4, 1, 3)
# plt.plot(sim_t, state_itt[:,2], 'r--', label="x_dot")
# plt.plot(sim_t, state_itt[:,3], 'k--', label="y_dot")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('Velocity(m/s)')
#
# plt.subplot(4, 1, 4)
# plt.plot(sim_t, state_itt[:,4], 'r--', label="theta")
# plt.legend()
# plt.xlabel('Time(s)')
# plt.ylabel('Pitch(degrees)')
#
# plt.subplots_adjust(left=0.1,
#                     bottom=0.1,
#                     right=0.9,
#                     top=0.9,
#                     wspace=0.4,
#                     hspace=0.4)
#
# plt.show()