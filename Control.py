# File for Control

class Gains:
    def __init__(self, kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

ContraHopperGains = Gains(kp = [0,0,0],
                          ki = [0,0,0] ,
                          kd = [0,0,0])

def control(C_B2I_target,C_B2I,ControlGains,prev_i,prev_error,user_control):
    """
    Contra Hopper's Control Algorithm
    """
    pid_output = np.array([0,0,0])

    return pid_output