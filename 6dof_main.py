import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import Kinematics

# Magic Numbers
import strapdown

in2m = 0.0254
kg2n = 9.8055

# Sim Options
sim_frequency = 500
start_time = 0
end_time = pi
isPlotting = False
is3d = False

dt = 1 / sim_frequency
sim_t = np.arange(start_time, end_time, dt)


# Initialization
class State:
    def __init__(self, position_m, velocity_mps, attitude_Cb2i_dcm, w_radps):
        self.position_m = np.array(position_m)
        self.velocity_mps = np.array(velocity_mps)
        self.attitude_Cb2i_dcm = np.array(attitude_Cb2i_dcm)
        self.w_radps = np.array(w_radps)


class Inputs:
    def __init__(self, forces_n, moments_nm):
        self.forces_n = np.array(forces_n)
        self.moments_nm = np.array(moments_nm)


class VehicleProperties:
    def __init__(self, mass_kg, i_tensor_cg):
        self.mass_kg = mass_kg
        self.i_tensor_cg = np.array(i_tensor_cg)


CurrentState = State(position_m=np.zeros(3), velocity_mps=np.zeros(3), attitude_Cb2i_dcm=np.identity(3),
                     w_radps=np.zeros(3))

CurrentInputs = Inputs(forces_n=np.zeros(3),
                       moments_nm=np.zeros(3))

ContraHopper = VehicleProperties(mass_kg=1, i_tensor_cg=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
Gyroscope = VehicleProperties(mass_kg=0.477, i_tensor_cg=[[0.447, 0, 0], [0, 1, 0], [0, 0, 0.477]])

g_mps2 = 0

# Data Collection
stash_array = np.zeros((len(sim_t), 14))

# Simulation
for i in sim_t:
    # if sim_t[i]-control_timer >= control_dt:
    CurrentState.w_radps = np.array([1, 0, 0])
    CurrentState.velocity_mps = np.array([0, 0, 0])

    Kinematics.get_new_state(CurrentState, CurrentInputs, g_mps2, ContraHopper ,dt)

print(CurrentState.attitude_Cb2i_dcm)
# print(strapdown.dcm2euler(CurrentState.attitude_Cb2i_dcm))
