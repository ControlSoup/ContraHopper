import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import kinematics

# Magic Numbers


in2m = 0.0254
kg2n = 9.8055

# Sim Options
sim_frequency = 1500
start_time = 0
end_time = pi
dt = 1 / sim_frequency
sim_t = np.arange(start_time, end_time, dt)
isPlotting = False
is3d = True


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


class Vehicle_Properties:
    def __init__(self, mass_kg, i_tensor_cg):
        self.mass_kg = mass_kg
        self.i_tensor_cg = np.array(i_tensor_cg)


Current_state = State(np.array([0, 0, 0]), np.array([0, 0, 0]),
                      np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
                      np.array([0, 0, 0]))

Current_inputs = Inputs(np.array([0, 0, 0]),
                        np.array([0, 0, 0]))

Contrahopper = Vehicle_Properties(1, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
g_mps2 = 0
stash_array = np.zeros((len(sim_t), 14))

# Simulation
for i in range(0, len(sim_t)):
    # if sim_t[i]-control_timer >= control_dt:
    Current_state.w_radps = np.array([0,0,0])
    Current_inputs.moments_npm = np.array([1, 0, 0])
    Current_inputs.forces_n = np.array([0, 0, 0])
    kinematics.new_state(Current_state, Current_inputs, dt, g_mps2, Contrahopper)

print(Current_state.attitude_Cb2i_dcm)
