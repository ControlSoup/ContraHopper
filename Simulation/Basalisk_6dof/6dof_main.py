import generic_plotting as gp
import os
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from copy import copy
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# import simulation related support
from Basilisk.simulation import spacecraft
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, unitTestSupport, vizSupport)

# always import the Basilisk messaging support
from Basilisk.architecture import messaging

def run(show_plots,sim_time,sim_frequency,log_frequency):

   # Create simulation variable names
   simTaskName = "simTask"
   simProcessName = "simProcess"

   #  Create a sim module as an empty container
   scSim = SimulationBaseClass.SimBaseClass()
   scSim.SetProgressBar(True)

   #  create the simulation process
   dynProcess = scSim.CreateNewProcess(simProcessName)

   # create the dynamics task and specify the integration update time
   simulationTimeStep = macros.sec2nano(1/sim_frequency)
   dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

   # setup the simulation tasks/objects
   scObject = spacecraft.Spacecraft()
   scObject.ModelTag = "bsk-Sat"

   # add spacecraft object to the simulation process
   scSim.AddModelToTask(simTaskName, scObject)

   scObject.hub.r_CN_NInit     = [[0.],[0.],[0.]]   # m   - r_BN_N
   scObject.hub.v_CN_NInit     = [[0.],[0.],[0.]]   # m/s - v_BN_N

   # specifiy simulation specifics
   simulationTime = macros.sec2nano(sim_time)
   numDataPoints = sim_time*log_frequency
   samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

   # create a logging task object of the spacecraft output message at the desired down sampling ratio
   dataRec = scObject.scStateOutMsg.recorder(samplingTime)
   scSim.AddModelToTask(simTaskName, dataRec)

   # initialize Simulation:  This function runs the self_init()
   # and reset() routines on each module.
   scSim.InitializeSimulation()

   # state refrences
   posRef   = scObject.dynManager.getStateObject("hubPosition")
   velRef   = scObject.dynManager.getStateObject("hubVelocity")
   omegaRef = scObject.dynManager.getStateObject("hubOmega")
   
   # get state
   #rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
   #vVt = unitTestSupport.EigenVector3d2np(velRef.getState())

   # operation on State on state
   new_vel = np.array([1,0,20.])
   new_omega = np.array([1,0,0.])

   # set state
   velRef.setState(unitTestSupport.np2EigenVectorXd(new_vel))
   omegaRef.setState(unitTestSupport.np2EigenVectorXd(new_omega))

   # configure a simulation stop time and execute the simulation run
   scSim.ConfigureStopTime(simulationTime)
   scSim.ExecuteSimulation()

   # retrieve the logged data
   posData  = dataRec.r_BN_N
   velData  = dataRec.v_BN_N
   mrpData  = dataRec.sigma_BN
   Cb2iData = dataRec.Cb2i

   np.set_printoptions(precision=16)

   if show_plots:
      gp.plot_position(dataRec.times(), posData)
      gp.plot_velocity(dataRec.times(), velData)
      gp.plot_attitude(dataRec.times(), mrpData)
      plt.show()
      gp.plot_trajectory(dataRec.times(),posData,Cb2iData)

   # close the plots being saved off to avoid over-writing old and new figures
   plt.close("all")


# Run the file 
if __name__ == "__main__":
    run(show_plots    = True, 
        sim_time      = 120,
        sim_frequency = 100, 
        log_frequency = 10)
