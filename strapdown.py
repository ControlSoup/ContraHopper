# the result matrix is replaced with euler angle equivalent of  a dcm (resulting angles in radians)
import numpy as np

rad2deg = 180 / np.pi


def dcm2euler(dcm):
    pitch = 0
    yaw = 0
    roll = 0
    roll = np.arctan(-dcm[2][0] / np.sqrt(1 - (dcm[2][0] * dcm[2][0])))

    if abs(dcm[2][0]) < 0.999:
        pitch = np.arctan(dcm[2][1] / dcm[2][2])
        yaw = np.arctan(dcm[1][0] / dcm[0][0])

    if dcm[2][0] <= - 0.999:
        pitch = yaw - np.arctan((dcm[1][2] - dcm[0][1]) / (dcm[0][2] + dcm[1][1]))

    if dcm[2][0] >= 0.999:
        pitch = np.pi + np.arctan((dcm[1][2] + dcm[0][1]) / (dcm[0][2] - dcm[1][1])) - yaw

    result = [pitch, yaw, roll]
    return result


# the result matrix is replaced with a dcm equivalent of three euler angles
def euler2dcm(euler=None, result=None):
    if euler is None:
        euler = [0, 0, 0]
    if result is None:
        result = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    [pitch, roll, yaw] = euler

    result[0][0] = np.cos(pitch) * np.cos(yaw)
    result[0][1] = -np.cos(roll) * np.sin(yaw) + (np.sin(roll) * np.sin(pitch) * np.sin(yaw))
    result[0][2] = np.sin(roll) * np.sin(yaw) + (np.cos(roll) * np.sin(pitch) * np.cos(yaw))
    result[1][0] = np.cos(pitch) * np.sin(yaw)
    result[1][1] = np.cos(roll) * np.cos(yaw) + (np.sin(roll) * np.sin(pitch) * np.sin(yaw))
    result[1][2] = -np.sin(roll) * np.cos(yaw) + (np.cos(roll) * np.sin(pitch) * np.sin(yaw))
    result[2][0] = -np.sin(pitch)
    result[2][1] = np.sin(roll) * np.cos(pitch)
    result[2][2] = np.cos(roll) * np.cos(pitch)
    return np.array(result)


def rates2dcm(dcm, rates):
    '''
    Inputs : angular rates, and the current dcm
    Outpus : a dcm rate
    '''
    # pg 3-52
    scew_sym = np.array([[0, -rates[2], rates[1]],
                         [rates[2], 0, -rates[0]],
                         [-rates[1], -rates[0], 0]])
    dcm_dot = np.matmul(dcm, scew_sym)

    return np.array(dcm_dot)


time = np.arange(0, 5, 0.0001)
i_prev = 0
rate = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
dis = 0
dis_dot = 0
integration = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])


for i in range(0, len(time)):
    temp = rates2dcm(integration, np.array([0.001, 0, 0]))
    print(dcm2euler(temp))
    integration = integration + (temp*0.0001)
    dis_dot = dis_dot + 0.001 * 0.0001
    dis = dis + dis_dot * 0.0001

euler = np.array(dcm2euler(integration))

print(euler)
print(dis)
