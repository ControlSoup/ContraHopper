import numpy as np
import Kinematics
import strapdown
from matplotlib import pyplot as plt
"""
===========================
Inputs
===========================
"""
# Sim Options
sim_frequency    = 500.0
start_time       = 0.0
end_time         = 10.0

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
# Pre-allocation of state_matrix (for plotting)
stash_position_m   = np.zeros((len(itt_sim),len(CurrentAbsoluteState.position_m)))
stash_velocity_mps = np.zeros((len(itt_sim),len(CurrentAbsoluteState.velocity_mps)))
stash_Cb2o_dcm_a   = np.zeros((len(itt_sim),len(CurrentAbsoluteState.Cb2i_dcm[0])))
stash_Cb2o_dcm_b   = np.zeros((len(itt_sim),len(CurrentAbsoluteState.Cb2i_dcm[0])))
stash_Cb2o_dcm_c   = np.zeros((len(itt_sim),len(CurrentAbsoluteState.Cb2i_dcm[0])))

"""
===========================
Sim Loop
===========================
"""
for i in range(len(itt_sim)):

    #Update State
    CurrentAbsoluteState.update_from_state_matrix(Kinematics.rk4(CurrentAbsoluteState.state_matrix, CurrentInputs, ContraHopper, dt))
    CurrentAbsoluteState.Cb2i_dcm = strapdown.orthonormalize(CurrentAbsoluteState.Cb2i_dcm) 

    # Store state as a state_matrix for plotting
    stash_position_m[i] = CurrentAbsoluteState.position_m
    stash_velocity_mps[i] = CurrentAbsoluteState.velocity_mps
    


"""
===========================
Data Presentation
===========================
"""
plot_2d          = False
plot_position_3d = False
print_final      = False

if plot_2d:
    fig, axs = plt.subplots(2)
    axs[0].plot(itt_sim, stash_position_m[:,0],label = "Actual X",c = "red")
    axs[0].plot(itt_sim, stash_position_m[:,1],label = "Actual Y",c = "green")
    axs[0].plot(itt_sim, stash_position_m[:,2],label = "Actual Z",c = "blue")
    axs[0].set_title("Position (m)")
    axs[0].legend()
    axs[1].plot(itt_sim, stash_velocity_mps[:,0],label = "Actual Xdot",c = "red")
    axs[1].plot(itt_sim, stash_velocity_mps[:,1],label = "Actual Ydot",c = "green")
    axs[1].plot(itt_sim, stash_velocity_mps[:,2],label = "Actual Zdot",c = "blue")
    axs[1].set_title("Velocity (m)")
    axs[1].legend()
    plt.show()

if plot_position_3d:
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(stash_position_m[:,0],stash_position_m[:,1],stash_position_m[:,2], label='Position (m)')
    ax.legend()
    plt.show()

if print_final:
    print(f'Final Position (m)         = {CurrentAbsoluteState.position_m}\n')
    print(f'Final Velocity (m/s)       = {CurrentAbsoluteState.velocity_mps}\n')
    print(f'Final Attitude (dcm)       = {CurrentAbsoluteState.Cb2i_dcm[0]}')
    print(f'                             {CurrentAbsoluteState.Cb2i_dcm[1]}')
    print(f'                             {CurrentAbsoluteState.Cb2i_dcm[2]}\n')
    print(f'Final Attitude (deg)       = {strapdown.dcm2euler(CurrentAbsoluteState.Cb2i_dcm)}\n')
    print(f'Final Angular Rate (rad/s) = {CurrentAbsoluteState.w_radps}')

print(CurrentAbsoluteState.state_matrix)
