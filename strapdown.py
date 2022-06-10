# the result matrix is replaced with euler angle equivalent of  a dcm (resulting angles in radians)
import numpy as np


def dcm2euler(dcm=None, result=[0, 0, 0]):
    if dcm is None:
        dcm = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    roll = np.atan(-dcm[2][0] / np.sqrt(1 - (dcm[2][0] * dcm[2][0])))

    if abs(dcm[2][0]) < 0.999:
        pitch = np.atan(dcm[2][1] / dcm[2][2])
        yaw = np.atan(dcm[1][0] / dcm[0][0])

    if dcm[2][0] <= - 0.999:
        pitch = yaw - np.atan((dcm[1][2] - dcm[0][1]) / (dcm[0][2] + dcm[1][1]))

    if dcm[2][0] >= 0.999:
        pitch = np.pi + np.atan((dcm[1][2] + dcm[0][1]) / (dcm[0][2] - dcm[1][1])) - yaw

    result[0] = pitch
    result[1] = roll
    result[2] = yaw
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
    return result

def gyro2dcm(gyro=None, Cb2i_gyro=None, dt):

  #scew symetric pg 3-52
  if gyro is None:
    gyro = [0, 0, 0]
  if Cb2i_gyro is None:
    Cb2i_gyro = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

  scew_sym = [[0,0,0],[0,0,0],[0,0,0]]
  scew_sym[0][0] = 0
  scew_sym[0][1] = -gyro[2]
  scew_sym[0][2] = gyro[1]

  scew_sym[1][0] = gyro[2]
  scew_sym[1][1] = 0
  scew_sym[1][2] = -gyro[0]

  scew_sym[2][0] = -gyro[1]
  scew_sym[2][1] = -gyro[0]
  scew_sym[2][2] = 0

  Cb2i_dot = np.cross(Cb2i_gyro, scew_sym)

  # gyro dcm estimate integration
  Cb2i_gyro[0][0] += Cb2i_dot[0][0] * dt
  Cb2i_gyro[1][0] += Cb2i_dot[1][0] * dt
  Cb2i_gyro[2][0] += Cb2i_dot[2][0] * dt

  Cb2i_gyro[0][1] += Cb2i_dot[0][1] * dt
  Cb2i_gyro[1][1] += Cb2i_dot[1][1] * dt
  Cb2i_gyro[2][1] += Cb2i_dot[2][1] * dt

  Cb2i_gyro[0][2] += Cb2i_dot[0][2] * dt
  Cb2i_gyro[1][2] += Cb2i_dot[1][2] * dt
  Cb2i_gyro[2][2] += Cb2i_dot[2][2] * dt

}