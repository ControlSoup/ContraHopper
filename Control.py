# File for Control
import strapdown


class TargetState:
    def __init__(self, target_position_m, target_velocity_mps, target_C_B2I, target_wdot_radps):
        self.target_position_m = target_position_m
        self.target_velocity_mps = target_velocity_mps
        self.target_C_B2I = target_C_B2I
        self.target_wdot_radps = target_wdot_radps


class ControlGains:
    def __init__(self, rate_kp, euler_attitude_kp, euler_attitude_ki):
        self.rate_kp = rate_kp
        self.euler_attitude_kp = euler_attitude_kp
        self.euler_attitude_ki = euler_attitude_ki


class ControlOutput:
    def __init__(self, position_output, velocity_output, rate_output, euler_attitude_output):
        self.position_output = position_output
        self.velocity_output = velocity_output
        self.rate_output = rate_output
        self.euler_attitude_output = euler_attitude_output


"""
===========================
Control Algorithm
===========================
"""


def control(TargetState, NavState, ControlGains, ControlOutput, prev_I, prev_rate_error, prev_C_B2I_error, user_control):
    """
    Contra Hopper's Control Algorithm
    """

    """
    Rate Controller
    """
    rate_error = [TargetState.target_wdot_radps[0] - NavState.w_radps[0],
                  TargetState.target_wdot_radps[1] - NavState.w_radps[1],
                  TargetState.target_wdot_radps[2] - NavState.w_radps[2]]
    prev_rate_error = rate_error

    ControlOutput.rate_output = [-rate_error[0] * ControlGains.rate_kp[0],
                                 -rate_error[1] * ControlGains.rate_kp[1],
                                 -rate_error[2] * ControlGains.rate_kp[2]]
    """
    Attitude Controller
    """

    error_C_B2I = strapdown.dcm_error(TargetState.target_C_B2I, NavState.C_B2I)

    prev_error_C_B2I = error_C_B2I

    euler_error = strapdown.dcm2euler(error_C_B2I)

