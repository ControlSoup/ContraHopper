import numpy as np
import Kinematics
import strapdown
import pathlib
from ctypes import *
from matplotlib import pyplot as plt
c_functions = CDLL(str(pathlib.Path(__file__).parent.resolve())+"..\..\..\FlightSoftware\\flight_software_sim.so")
c_functions.flight_software_sim.restype = POINTER(c_double)
c_functions.flight_software_sim.argtypes = [POINTER(c_double),c_double,c_double]
"""
===========================
Inputs
===========================
"""
# Sim Options
sim_frequency     = 500.0
start_time        = 0.0
end_time          = 10.0

# State Variables 
init_position_m   = np.array([0,0,0.])
init_velocity_mps = np.array([1,0,0.])
init_Cb2i_dcm     = np.identity(3)
init_w_radps      = np.array([0,0,0.])

# Inputs
init_forces_n     = np.zeros(3)
init_moments_nm   = np.zeros(3)

"""
===========================
Initialization
===========================
""" 


CurrentAbsoluteState = Kinematics.State(position_m   = init_position_m, 
                                        velocity_mps = init_velocity_mps,
                                        Cb2i_dcm     = init_Cb2i_dcm,
                                        w_radps      = init_w_radps)

CurrentInputs = Kinematics.Inputs(forces_n = init_forces_n,
                                  moments_nm = init_moments_nm)

ContraHopper = Kinematics.MassProperties(mass_kg=10, 
                                         i_tensor_cg=[[1, 0, 0], 
                                                      [0, 1, 0], 
                                                      [0, 0, 1.]])

Gyroscope = Kinematics.MassProperties(mass_kg=0.26, 
                                      i_tensor_cg=[[0.7,0,0.7], 
                                                   [0.0,1,0.0], 
                                                   [0.7,0,-0.7]])
dt = 1/sim_frequency
itt_sim = np.arange(start_time,end_time,dt)

"""
===========================
Data Collection
=========================== 
"""
# Pre-allocation of state_vector (for plotting)
stash_state_vector = np.zeros((len(itt_sim),len(CurrentAbsoluteState.state_vector)))


"""
===========================
Sim Loop
===========================
"""
for i in range(len(itt_sim)):

    #Update State with rk4 integration
    CurrentAbsoluteState.update_from_state_vector(Kinematics.rk4(CurrentAbsoluteState.state_vector, CurrentInputs, ContraHopper, dt))

    # Store state as a state_vector for plotting
    stash_state_vector[i] = CurrentAbsoluteState.state_vector
"""
===========================
Data Presentation
===========================
"""
plot_sim_raw_3d  = False
plot_sim_raw_2d  = True
plot_position_3d = False
plot_position_2d = False
print_final      = False

if plot_sim_raw_3d:
    fig, axs = plt.subplots(2)
    axs[0].plot(itt_sim, stash_state_vector[:,0],label = "Actual X",c = "red")
    axs[0].plot(itt_sim, stash_state_vector[:,1],label = "Actual Y",c = "green")
    axs[0].plot(itt_sim, stash_state_vector[:,2],label = "Actual Z",c = "blue")
    axs[0].set_title("Position (m)")
    axs[0].legend()
    axs[1].plot(itt_sim, stash_state_vector[:,3],label = "Actual Xdot",c = "red")
    axs[1].plot(itt_sim, stash_state_vector[:,4],label = "Actual Ydot",c = "green")
    axs[1].plot(itt_sim, stash_state_vector[:,5],label = "Actual Zdot",c = "blue")
    axs[1].set_title("Velocity (m)")
    axs[1].legend()
    plt.show()

if plot_sim_raw_2d:
    fig, axs = plt.subplots(2)
    axs[0].plot(itt_sim, stash_state_vector[:,0],label = "Actual X",c = "red")
    axs[0].plot(itt_sim, stash_state_vector[:,1],label = "Actual Y",c = "green")
    axs[0].set_title("Position (m)")
    axs[0].legend()
    axs[1].plot(itt_sim, stash_state_vector[:,3],label = "Actual Xdot",c = "red")
    axs[1].plot(itt_sim, stash_state_vector[:,4],label = "Actual Ydot",c = "green")
    axs[1].set_title("Velocity (m)")
    axs[1].legend()
    plt.show()

if plot_position_3d:
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(stash_state_vector[:,0],stash_state_vector[:,1],stash_state_vector[:,2], label='Position (m)')
    ax.legend()
    plt.show()

if plot_position_2d:
    plt.plot(stash_state_vector[:,0],stash_state_vector[:,1],label = "Position (m)")
    plt.legend()
    plt.show()

if print_final:
    print(f'Final Position (m)         = {CurrentAbsoluteState.position_m}\n')
    print(f'Final Velocity (m/s)       = {CurrentAbsoluteState.velocity_mps}\n')
    print(f'Final Attitude (dcm)       = {CurrentAbsoluteState.Cb2i_dcm[0]}')
    print(f'                             {CurrentAbsoluteState.Cb2i_dcm[1]}')
    print(f'                             {CurrentAbsoluteState.Cb2i_dcm[2]}\n')
    print(f'Final Attitude (deg)       = {strapdown.dcm2euler(CurrentAbsoluteState.Cb2i_dcm)}\n')
    print(f'Final Angular Rate (rad/s) = {CurrentAbsoluteState.w_radps}')


# Array passing to a C function Test (works)
# array = np.array([0,0,0,0,0,0],dtype=c_double)
# array = array.ctypes.data_as(POINTER(c_double))
# array_result = c_functions.flight_software_sim(array,10.0,1.0)
# array_result = np.array([array_result[0],array_result[1],array_result[2],
#                          array_result[3],array_result[4],array_result[5]])
# print(array_result)

