import numpy as np
import kinematics
import flightsoftware_model
import strapdown
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from unit_conversions import deg2rad,rad2deg


"""
===========================
Inputs
===========================
"""
# Sim Options
sim_frequency           = 250
start_time              = 0.0
end_time                = 10.0

# State Variables 
init_position_m        = np.array([0,0,0.])
init_velocity_mps      = np.array([0,0,0.])
init_acceleration_mps2 = np.array([0,0,0])
init_Cb2i_dcm          = np.array([[1,0,0],
                              [0,1,0],
                              [0,0,1.]])
init_w_radps           = np.array([0,0,0])
     
# Inputs     
init_forces_n          = np.array([0,0,0])
init_moments_nm        = np.array([0,0,0])

# Controller Options
control_frequency      = 50.

np.random.seed(400)

"""
===========================
Initialization
===========================
""" 

CurrentAbsoluteState = kinematics.State(position_m        = init_position_m, 
                                        velocity_mps      = init_velocity_mps,
                                        acceleration_mps2 = init_acceleration_mps2,
                                        Cb2i_dcm          = init_Cb2i_dcm,
                                        w_radps           = init_w_radps)

CurrentInputs = kinematics.Inputs(forces_n = init_forces_n,
                                  moments_nm = init_moments_nm)

ContraHopper = kinematics.MassProperties(mass_kg = 10.0, 
                                         i_tensor_cg=[[1, 0, 0], 
                                                      [0, 1, 0], 
                                                      [0, 0, 1.]])

Gyroscope = kinematics.MassProperties(mass_kg = 0.26, 
                                      i_tensor_cg=[[0.7,0,0.7], 
                                                   [0.0,1,0.0], 
                                                   [0.7,0,-0.7]])

last_control_update = 0
control_dt          = 1/control_frequency
dt                  = 1/sim_frequency
itt_sim             = np.arange(start_time,end_time,dt)

"""
===========================
Data Collection
=========================== 
"""

# Pre-allocation of state_vector (for plotting)
stash_state_vector = np.zeros((len(itt_sim),len(CurrentAbsoluteState.state_vector)))
stash_input_vector = np.zeros((len(itt_sim),len(CurrentInputs.input_vector)))
stash_position_controller = np.zeros((len(itt_sim),3))

"""
===========================
Sim Loop
===========================
"""

for i in range(len(itt_sim)):

    # Update State with rk4 integration, and orthonormalize the attitude
    CurrentAbsoluteState.update_from_state_vector(kinematics.rk4(CurrentAbsoluteState.state_vector, CurrentInputs, ContraHopper, dt))
    CurrentAbsoluteState.orthonomormalize_Cb2i_dcm()
    
    # Store state as a state_vector for plotting
    print(CurrentAbsoluteState.state_vector)
    stash_input_vector[i] = CurrentInputs.input_vector
    stash_position_controller[i] = np.array(flightsoftware_model.PositionController.pid_I)
# Useful stats
max_position_m_vector = np.array([max(stash_state_vector[:,0]),max(stash_state_vector[:,1]),max(stash_state_vector[:,2])])
min_position_m_vector = np.array([min(stash_state_vector[:,0]),min(stash_state_vector[:,1]),min(stash_state_vector[:,2])])

"""
===========================
Data Presentation
===========================
"""
# Input plotting
plot_inputs      = True
plot_I           = True

# Sensor plotting

# State plotting
print_final      = False
plot_position_3d = False
plot_raw_2d      = False
plot_position_2d = False
plot_trajectory  = True
plot_raw_3d      = False

if plot_inputs:
    fig, axs = plt.subplots(2)
    axs[0].plot(itt_sim, stash_input_vector[:,0],label = "X Force",c = "red")
    axs[0].plot(itt_sim, stash_input_vector[:,1],label = "Y Force",c = "green")
    axs[0].plot(itt_sim, stash_input_vector[:,2],label = "Z Force",c = "blue")
    axs[0].set_title("Force (N)")
    axs[0].legend()
    axs[1].plot(itt_sim, stash_input_vector[:,3],label = "X Moment",c = "red")
    axs[1].plot(itt_sim, stash_input_vector[:,4],label = "Y Moment",c = "green")
    axs[1].plot(itt_sim, stash_input_vector[:,5],label = "Z Moment",c = "blue")
    axs[1].set_title("Moments (Nm)")
    axs[1].legend()
    plt.show()

if plot_I:
    fig, axs = plt.subplots(2)
    axs[0].plot(itt_sim, stash_position_controller[:,0],label = "X pid_I",c = "red")
    axs[0].plot(itt_sim, stash_position_controller[:,1],label = "Y pid_I",c = "red")
    axs[0].plot(itt_sim, stash_position_controller[:,2],label = "Z pid_I",c = "red")
    axs[0].set_title("Pid_I (N)")
    axs[0].legend()
    plt.show()

if plot_raw_3d:
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

if plot_raw_2d:
    fig, axs = plt.subplots(2)
    axs[0].plot(itt_sim, stash_state_vector[:,0],label = "Actual X",c = "red")
    axs[0].plot(itt_sim, stash_state_vector[:,2],label = "Actual Z",c = "green")
    axs[0].set_title("Position (m)")
    axs[0].legend()
    axs[1].plot(itt_sim, stash_state_vector[:,3],label = "Actual Xdot",c = "red")
    axs[1].plot(itt_sim, stash_state_vector[:,5],label = "Actual Zdot",c = "green")
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
    print(f'Final Position    (m)      = {CurrentAbsoluteState.position_m}\n')
    print(f'Final Velocity    (m/s)    = {CurrentAbsoluteState.velocity_mps}\n')
    print(f'Final Attitude    (dcm)    = {CurrentAbsoluteState.Cb2i_dcm[0]}')
    print(f'                             {CurrentAbsoluteState.Cb2i_dcm[1]}')
    print(f'                             {CurrentAbsoluteState.Cb2i_dcm[2]}\n')
    print(f'Final Attitude     (rad)   = {strapdown.dcm2euler(CurrentAbsoluteState.Cb2i_dcm)}\n')
    print(f'Final Angular Rate (rad/s) = {CurrentAbsoluteState.w_radps}')

if plot_trajectory:
    # Set plotting frequency
    plot_frequency = 10
    axis_scale     = 5

    # Convert the sim time to the new plotting frequency
    itt_plot       = itt_sim[0:len(stash_state_vector):int(sim_frequency/plot_frequency)]

    # Convert state_vector to a plot vector at the new frequency
    plot_data      = stash_state_vector[0:len(stash_state_vector):int(sim_frequency/plot_frequency),:] 
    
    max_position_m = max(max_position_m_vector)
    min_position_m = min(min_position_m_vector)

    def animate(index):
        # Source: https://towardsdatascience.com/how-to-animate-plots-in-python-2512327c8263
        ax.clear()  # Clears the figure to update the line, point,   
                    # title, and axes

        # Updating Trajectory Line (index+1 due to Python indexing)
        ax.plot3D(plot_data[:index+1, 0], 
                  plot_data[:index+1, 1], 
                  plot_data[:index+1, 2], 
                  c='black')

        # Updating Point Location 
        ax.scatter(plot_data[index, 0], 
                   plot_data[index, 1], 
                   plot_data[index, 2], 
                   c='black', marker='o')
        # Origin
        # Adding Constant Origin X axis
        ax.plot3D([plot_data[0, 0],(plot_data[0, 0] + max_position_m/axis_scale)],
                  [plot_data[0, 1],plot_data[0, 1]], 
                  [plot_data[0, 2],plot_data[0, 2]],     
                  c='red')

        # Adding Constant Origin Y axis
        ax.plot3D([plot_data[0, 0],plot_data[0, 0]],
                  [plot_data[0, 1],(plot_data[0, 1] + max_position_m/axis_scale)], 
                  [plot_data[0, 2],plot_data[0, 2]],     
                  c='green')

        # Adding Constant Origin z axis
        ax.plot3D([plot_data[0, 0],plot_data[0, 0]],
                  [plot_data[0, 1],plot_data[0, 1]], 
                  [plot_data[0, 2],(plot_data[0, 2] + max_position_m/axis_scale)],     
                  c='blue')
        

        # Oreientation self.state_vector[6:9]
        # Cb2i [0]
        ax.plot3D([plot_data[index,0],(plot_data[index,0]+(plot_data[index,  6]*max_position_m/axis_scale))],
                  [plot_data[index,1],(plot_data[index,1]+(plot_data[index,  7]*max_position_m/axis_scale))],
                  [plot_data[index,2],(plot_data[index,2]+(plot_data[index,  8]*max_position_m/axis_scale))],    
                  c='red')
        
        # Cb2i [1]
        ax.plot3D([plot_data[index,0],(plot_data[index,0]+(plot_data[index,  9]*max_position_m/axis_scale))],
                  [plot_data[index,1],(plot_data[index,1]+(plot_data[index, 10]*max_position_m/axis_scale))],
                  [plot_data[index,2],(plot_data[index,2]+(plot_data[index, 11]*max_position_m/axis_scale))],    
                  c='green')
        
        # Cb2i [2]
        ax.plot3D([plot_data[index,0],(plot_data[index,0]+(plot_data[index, 12]*max_position_m/axis_scale))],
                  [plot_data[index,1],(plot_data[index,1]+(plot_data[index, 13]*max_position_m/axis_scale))],
                  [plot_data[index,2],(plot_data[index,2]+(plot_data[index, 14]*max_position_m/axis_scale))],    
                  c='blue')

        # Setting Axes Limits
        ax.set_xlim3d([min_position_m, max_position_m])
        ax.set_ylim3d([min_position_m, max_position_m])
        ax.set_zlim3d([min_position_m, max_position_m])

        # Adding Figure Labels
        ax.set_title('Trajectory \nTime = ' + str(np.round(itt_plot[index],    
                     decimals=2)) + ' sec')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
    
    # Plotting the Animation
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    line_ani = animation.FuncAnimation(fig, animate, interval=1,   
                                    frames=len(itt_plot))
    plt.show()
