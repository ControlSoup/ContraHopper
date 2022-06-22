# File for Control
import numpy as np
import strapdown


class TargetState:
    def __init__(self, target_position_m, target_velocity_mps, target_C_B2I, target_wdot_radps):
        self.target_position_m = target_position_m
        self.target_velocity_mps = target_velocity_mps
        self.target_C_B2I = target_C_B2I
        self.target_wdot_radps = target_wdot_radps


class Gains:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd


ContraHopperGains = Gains(kp=[0, 0, 0],
                          ki=[0, 0, 0],
                          kd=[0, 0, 0])

"""
===========================
Control Algorithm
===========================
"""


def control(TargetState, NavState, ControlGains, prev_I, prev_error, user_control):
    """
    Contra Hopper's Control Algorithm
    """

    """
    Target and Nav prasing
    """
    target_position_m = TargetState.target_position_m
    target_velocity_mps = TargetState.target_velocity_mps
    target_C_B2I = TargetState.target_C_B2I
    target_wdot_radps = TargetState.target_wdot_radps

    position_m = NavState.position_m
    velocity_mps = NavState.velocity_mps
    nav_C_B2I = NavState.attitude_Cb2i_dcm
    wdot_radps = NavState.wdot_radps



    """
    Rate Controller
    """
    rate_error = [target_wdot_radps[0] - wdot_radps[0],
                  target_wdot_radps[1] - wdot_radps[1],
                  target_wdot_radps[2] - wdot_radps[2]]
    prev_rate_error = rate_error

    rate_output = [-rate_error[0] * ControlGains.kd[0],
                   -rate_error[1] * ControlGains.kd[1],
                   -rate_error[2] * ControlGains.kd[2]]
    """
    Attitude Controller
    """

    error_C_B2I = strapdown.dcm_error(target_C_B2I, nav_C_B2I)

    prev_error_C_B2I = error_C_B2I

    euler_error = strapdown.dcm2euler(error_C_B2I)

    return rate_output
