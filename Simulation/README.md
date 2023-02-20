# Simulation
This folder contains all the files used to develop the flight software for ContraHopper. Orginally, I worked to develop a 6dof simulation, in python based on my own kinematics and intergration implementations (see the archive folder). I have swapped over to the basalisk framework instead to take advatange of its Cpp/C module integrations (somthing I was attempting with the package "Ctypes") and much faster run time. 

Information about the Basalisk simulation frame work can be found at: http://hanspeterschaub.info/basilisk/
## Current Sim
- Intilizes basalisk frame work
- Creates a spacecraft object
- Performs a constant velocity and constant angular velocity movement
- Plots data and trajectory

## To Dos:
- Implement forces and torques on the spacecraft Object
- Implement a FSW module that will integrate with the C files written in FlighSoftware

