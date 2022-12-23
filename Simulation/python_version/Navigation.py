# File for navigation
import strapdown


class NavState:
    def __init__(self, position_m, velocity_mps,acceleration_mps2, C_B2I, w_radps):
        self.position_m = np.array(position_m)
        self.velocity_mps = np.array(velocity_mps)
        self.acceleration_mps2 = acceleration_mps2
        self.C_B2I = np.array(C_B2I)
        self.w_radps = np.array(w_radps)



def navigation(NavState, Sensors, dt):

    NavState.w_radps = Sensors.gyro

    Cdot_B2I = strapdown.rates2dcm(NavState.C_B2I, Sensors.gyro)

    NavState.C_B2I = NavState.C_B2I + (Cdot_B2I * dt)

    NavState.acceleration_mps2 = Sensors.accelerometer

    NavState.velocity_mps = NavState.velocity_mps + (Sensors.accelerometer * dt)

    NavState.position_m = NavState.position_m + (NavState.velocity_mps * dt)
