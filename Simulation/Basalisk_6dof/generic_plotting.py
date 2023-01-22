import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation


def plot_trajectory(time_ns,posData,Cb2iData,axis_scale = 0.1,playback_delay = 1):
    time_s = time_ns / 10**9 
    max_position_m = max(max(posData[:,0]),max(posData[:,1]),max(posData[:,2]))
    min_position_m = min(min(posData[:,0]),min(posData[:,1]),min(posData[:,2]))

    def animate(index):
        # Source: https://towardsdatascience.com/how-to-animate-plots-in-python-2512327c8263
        ax.clear()  # Clears the figure to update the line, point,   
                    # title, and axes

        # Updating Trajectory Line (index+1 due to Python indexing)
        ax.plot3D(posData[:index+1, 0], 
                  posData[:index+1, 1], 
                  posData[:index+1, 2], 
                  c='black')

        # Updating Point Location 
        ax.scatter(posData[index, 0], 
                   posData[index, 1], 
                   posData[index, 2], 
                   c='black', marker='o')
        # Origin
        # Adding Constant Origin X axis
        ax.plot3D([posData[0, 0],(posData[0, 0] + max_position_m * axis_scale)],
                  [posData[0, 1], posData[0, 1]], 
                  [posData[0, 2], posData[0, 2]],     
                  c='red')

        # Adding Constant Origin Y axis
        ax.plot3D([posData[0, 0], posData[0, 0]],
                  [posData[0, 1],(posData[0, 1] + max_position_m * axis_scale)], 
                  [posData[0, 2], posData[0, 2]],     
                  c='green')

        # Adding Constant Origin z axis
        ax.plot3D([posData[0, 0], posData[0, 0]],
                  [posData[0, 1], posData[0, 1]], 
                  [posData[0, 2],(posData[0, 2] + max_position_m * axis_scale)],     
                  c='blue')
        

        # Oreientation self.state_vector[6:9]
        # Cb2i [0]
        ax.plot3D([posData[index,0],(posData[index,0]+(Cb2iData[index, 0]*max_position_m * axis_scale))],
                  [posData[index,1],(posData[index,1]+(Cb2iData[index, 1]*max_position_m * axis_scale))],
                  [posData[index,2],(posData[index,2]+(Cb2iData[index, 2]*max_position_m * axis_scale))],    
                  c='red')

        # Cb2i [1]
        ax.plot3D([posData[index,0],(posData[index,0]+(Cb2iData[index, 3]*max_position_m * axis_scale))],
                  [posData[index,1],(posData[index,1]+(Cb2iData[index, 4]*max_position_m * axis_scale))],
                  [posData[index,2],(posData[index,2]+(Cb2iData[index, 5]*max_position_m * axis_scale))],    
                  c='green')
        
        # Cb2i [2]
        ax.plot3D([posData[index,0],(posData[index,0]+(Cb2iData[index, 6]*max_position_m * axis_scale))],
                  [posData[index,1],(posData[index,1]+(Cb2iData[index, 7]*max_position_m * axis_scale))],
                  [posData[index,2],(posData[index,2]+(Cb2iData[index, 8]*max_position_m * axis_scale))],    
                  c='blue')

        # Setting Axes Limits
        ax.set_xlim3d([min_position_m, max_position_m])
        ax.set_ylim3d([min_position_m, max_position_m])
        ax.set_zlim3d([min_position_m, max_position_m])

        # Adding Figure Labels
        ax.set_title('Trajectory \nTime = ' + str(np.round(time_s[index],decimals=2)) + ' sec')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
    
    # Plotting the Animation
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    line_ani = animation.FuncAnimation(fig, animate, interval=playback_delay, frames=len(time_s))
    plt.show()

def plot_position(time_ns, posData):
    time_s = time_ns / (10**9) 
    plt.figure(1)
    for i in range(3):
        if i == 0:
            plot_name = 'X position'
        elif i == 1:
            plot_name = 'Y position'
        else:
            plot_name = 'Z position'
        
        plt.plot(time_s,posData[:,i], label = plot_name)
    plt.legend(loc='lower left')
    plt.xlabel('Time (s)')
    plt.ylabel('Meters')
    plt.title('Inertial Position')
    
def plot_velocity(time_ns, velData):
    time_s = time_ns / (10**9) 
    plt.figure(2)
    for i in range(3):
        if i == 0:
            plot_name = 'X velocity'
        elif i == 1:
            plot_name = 'Y velocity'
        else:
            plot_name = 'Z velocity'
        
        plt.plot(time_s,velData[:,i],
                 label = plot_name)
    plt.legend(loc='lower left')
    plt.xlabel('Time (s)')
    plt.ylabel('Meters')
    plt.title('Inertial Velocity')

def plot_attitude(time_ns, mrpData):
    time_s = time_ns / (10**9) 
    plt.figure(3)
    for i in range(3):
        if i == 0:
            plot_name = 'X Theta'
        elif i == 1:
            plot_name = 'Y Psi'
        else:
            plot_name = 'Z Phi'
        
        plt.plot(time_s,mrpData[:,i], label = plot_name)
    plt.legend(loc='lower left')
    plt.xlabel('Time (s)')
    plt.ylabel('Radians')
    plt.title('Attitude')
