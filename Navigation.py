# File for navigation
import strapdown


def navigation(C_B2I,gyro,accelerometer,dt):

    Cdot_B2I = strapdown.rates2dcm(C_B2I,gyro)
    C_B2I = C_B2I + C_B2I*dt