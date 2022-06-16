import numpy as np
from numpy import sqrt, pi, sin, cos, tan, arctan, matmul, identity, transpose
from numpy.linalg import inv


def dcm2euler(dcm):
    psi = arctan(-dcm[2, 0] / sqrt(dcm[2, 1] ** 2 + dcm[2, 2] ** 2))

    if abs(dcm[2][0]) < 0.999:
        theta = arctan(dcm[2, 1] / dcm[2, 2])
        phi = arctan(dcm[1, 0] / dcm[0, 0])
    else:
        return "Unable to produce complete Euler angles >2pi"
    return np.array([theta, psi, phi])


# the result matrix is replaced with a dcm equivalent of three euler angles
def euler2dcm(euler):
    if euler is None:
        euler = [0, 0, 0]

    [pitch, roll, yaw] = euler
    result = identity(3)
    result[0][0] = cos(pitch) * np.cos(yaw)
    result[0][1] = -np.cos(roll) * np.sin(yaw) + (np.sin(roll) * np.sin(pitch) * np.sin(yaw))
    result[0][2] = np.sin(roll) * np.sin(yaw) + (np.cos(roll) * np.sin(pitch) * np.cos(yaw))

    result[1][0] = np.cos(pitch) * np.sin(yaw)
    result[1][1] = np.cos(roll) * np.cos(yaw) + (np.sin(roll) * np.sin(pitch) * np.sin(yaw))
    result[1][2] = -np.sin(roll) * np.cos(yaw) + (np.cos(roll) * np.sin(pitch) * np.sin(yaw))

    result[2][0] = -np.sin(pitch)
    result[2][1] = np.sin(roll) * np.cos(pitch)
    result[2][2] = np.cos(roll) * np.cos(pitch)
    return np.array(result)


def rates2dcm(dcm, w):
    '''
    Inputs : angular rates (w), and the current dcm
    Outpus : a dcm rate (dcm_dot)
    '''
    # pg 3-52
    scew_sym = np.array([[0, -w[2], w[1]],
                         [w[2], 0, -w[0]],
                         [-w[1], w[0], 0]])

    dcm_dot = matmul(dcm, scew_sym)

    return np.array(dcm_dot)


def orthonormalize(Cbl_minus):
    # pg  7-19

    Esym = (matmul(Cbl_minus, transpose(Cbl_minus)) - identity(3)) / 2  # Error in the rows of the dcm

    Cbl_plus = matmul(identity(3) - Esym, Cbl_minus)

    return Cbl_plus

# 7.1.1.3-1 and 7.1.1.3-9 (7.1.1.3 7-18)
