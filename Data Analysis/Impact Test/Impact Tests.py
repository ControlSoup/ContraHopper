from pandas import read_csv
import matplotlib.pyplot as plt
import numpy as np

# Global

standard_gravity_mps2 = 9.8055
#model functions

def estimate_delta_t(a,h):
    return np.sqrt(2*standard_gravity_mps2*h)/a

#m\frac{\sqrt{2gh}}{\Delta t}
def estimate_force (m,h,t):
    return m * np.sqrt(2*standard_gravity_mps2*h)/t

# 6in_test_1
data = read_csv('6in_test_1.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

in6_test_1_time_s = np.zeros(len(time_str))
in6_test_1_amag_mps2 = np.zeros(len(time_str))
for i in range(0, len(time_str)):
    in6_test_1_time_s[i] = float(time_str[i]) - float(time_str[0])
    in6_test_1_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(in6_test_1_time_s)):
    if in6_test_1_time_s[i] >= 7 < 8.5:
        imu_scale.append(in6_test_1_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
in6_test_1_amag_mps2 = in6_test_1_amag_mps2 * mean_correction

# getmax
in6_test_1_acc_max = max(in6_test_1_amag_mps2)

# 6in_test_2
data = read_csv('6in_test_2.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

in6_test_2_time_s = np.zeros(len(time_str))
in6_test_2_amag_mps2 = np.zeros(len(time_str))
for i in range(0, len(time_str)):
    in6_test_2_time_s[i] = float(time_str[i]) - float(time_str[0])
    in6_test_2_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(in6_test_2_time_s)):
    if in6_test_2_time_s[i] >= 6.2 < 7:
        imu_scale.append(in6_test_2_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
in6_test_2_amag_mps2 = in6_test_2_amag_mps2 * mean_correction

# Get Max

in6_test_2_acc_max = max(in6_test_2_amag_mps2)

# 6in_test_3
data = read_csv('6in_test_3.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

in6_test_3_time_s = np.zeros(len(time_str))
in6_test_3_amag_mps2 = np.zeros(len(time_str))

for i in range(0, len(time_str)):
    in6_test_3_time_s[i] = float(time_str[i]) - float(time_str[0])
    in6_test_3_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(in6_test_3_time_s)):
    if in6_test_3_time_s[i] >= 0.2 < 1.2:
        imu_scale.append(in6_test_3_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
in6_test_3_amag_mps2 = in6_test_3_amag_mps2 * mean_correction

# Get Max
in6_test_3_acc_max = max(in6_test_3_amag_mps2)


# 1ft_test_1
data = read_csv('1ft_test_1.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

ft1_test_1_time_s = np.zeros(len(time_str))
ft1_test_1_amag_mps2 = np.zeros(len(time_str))

for i in range(0, len(time_str)):
    ft1_test_1_time_s[i] = float(time_str[i]) - float(time_str[0])
    ft1_test_1_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(ft1_test_1_time_s)):
    if ft1_test_1_time_s[i] >= 0.2 < 0.9:
        imu_scale.append(ft1_test_1_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
ft1_test_1_amag_mps2 = ft1_test_1_amag_mps2 * mean_correction

# Get Max
ft1_test_1_acc_max = max(ft1_test_1_amag_mps2)

# 1ft_test_2
data = read_csv('1ft_test_2.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

ft1_test_2_time_s = np.zeros(len(time_str))
ft1_test_2_amag_mps2 = np.zeros(len(time_str))

for i in range(0, len(time_str)):
    ft1_test_2_time_s[i] = float(time_str[i]) - float(time_str[0])
    ft1_test_2_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(ft1_test_2_time_s)):
    if ft1_test_2_time_s[i] >= 0.2 < 0.9:
        imu_scale.append(ft1_test_2_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
ft1_test_2_amag_mps2 = ft1_test_2_amag_mps2 * mean_correction

# Get Max
ft1_test_2_acc_max = max(ft1_test_2_amag_mps2)


# 1ft_test_3
data = read_csv('1ft_test_3.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

ft1_test_3_time_s = np.zeros(len(time_str))
ft1_test_3_amag_mps2 = np.zeros(len(time_str))

for i in range(0, len(time_str)):
    ft1_test_3_time_s[i] = float(time_str[i]) - float(time_str[0])
    ft1_test_3_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(ft1_test_3_time_s)):
    if ft1_test_3_time_s[i] >= 0.2 < 0.9:
        imu_scale.append(ft1_test_3_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
ft1_test_3_amag_mps2 = ft1_test_3_amag_mps2 * mean_correction

# Get Max
ft1_test_3_acc_max = max(ft1_test_3_amag_mps2)



# 2ft_test_1
data = read_csv('2ft_test_1.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

ft2_test_1_time_s = np.zeros(len(time_str))
ft2_test_1_amag_mps2 = np.zeros(len(time_str))

for i in range(0, len(time_str)):
    ft2_test_1_time_s[i] = float(time_str[i]) - float(time_str[0])
    ft2_test_1_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(ft2_test_1_time_s)):
    if ft2_test_1_time_s[i] >= 0.2 < 0.9:
        imu_scale.append(ft2_test_1_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
ft2_test_1_amag_mps2 = ft2_test_1_amag_mps2 * mean_correction

# Get Max
ft2_test_1_acc_max = max(ft2_test_1_amag_mps2)

# 2ft_test_1
data = read_csv('2ft_test_2.txt', sep=',', skiprows=0)
test_number_str = data['test_number']
time_str = data['time(s)']
ax_str = data['ax(ms2)']
ay_str = data['ay(ms2)']
az_str = data['az(ms2)']

ft2_test_2_time_s = np.zeros(len(time_str))
ft2_test_2_amag_mps2 = np.zeros(len(time_str))

for i in range(0, len(time_str)):
    ft2_test_2_time_s[i] = float(time_str[i]) - float(time_str[0])
    ft2_test_2_amag_mps2[i] = np.sqrt(float(ax_str[i]) ** 2 + float(ay_str[i]) ** 2 + float(az_str[i]) ** 2)

# IMU scaling
imu_scale = []
for i in range(0, len(ft2_test_2_time_s)):
    if ft2_test_2_time_s[i] >= 0.2 < 1.2:
        imu_scale.append(ft2_test_2_amag_mps2[i])

mean_correction = standard_gravity_mps2 / (sum(imu_scale) / len(imu_scale))

# Corrects level acceleration magntitudes to standard gravity
ft2_test_2_amag_mps2 = ft2_test_2_amag_mps2 * mean_correction

# Get Max
ft2_test_2_acc_max = max(ft2_test_2_amag_mps2)

# m=0.960
# print("6in Max Accel (mps2):")
# print(in6_test_1_acc_max)
# print(in6_test_2_acc_max)
# print(in6_test_3_acc_max)
# print()
# print("6in Force (N):")
# print(in6_test_1_acc_max*m)
# print(in6_test_2_acc_max*m)
# print(in6_test_3_acc_max*m)
# print()
# print("6in Model Estimated t:")
# print(estimate_delta_t(in6_test_1_acc_max,0.1524))
# print(estimate_delta_t(in6_test_2_acc_max,0.1524))
# print(estimate_delta_t(in6_test_3_acc_max,0.1524))
# print()
# print("1ft Max Accel (mps2):")
# print(ft1_test_1_acc_max)
# print(ft1_test_2_acc_max)
# print(ft1_test_3_acc_max)
# print('')
# print("1ft  F:")
# print(ft1_test_1_acc_max*m)
# print(ft1_test_2_acc_max*m)
# print(ft1_test_3_acc_max*m)
# print()
# print("1ft Model Estimated t:")
# print(estimate_delta_t(ft1_test_1_acc_max,0.3048))
# print(estimate_delta_t(ft1_test_2_acc_max,0.3048))
# print(estimate_delta_t(ft1_test_3_acc_max,0.3048))
# print()
# print("2ft Max Accel (mps2):")
# print(ft2_test_1_acc_max)
# print(ft2_test_2_acc_max)
# print()
# print("2ft F (N):")
# print(ft2_test_1_acc_max*m)
# print(ft2_test_2_acc_max*m)
# print()
# print("2ft Model Estimated t:")
# print(estimate_delta_t(ft2_test_1_acc_max,2*0.3048))
# print(estimate_delta_t(ft2_test_2_acc_max,2*0.3048))
# print()

plt.title('0.96kg Drone Dropped from 0.1524m')
plt.plot(in6_test_1_time_s, in6_test_1_amag_mps2, c='b', label = 'Test 1')
plt.plot(in6_test_2_time_s, in6_test_2_amag_mps2, c='g', label = 'Test 2')
plt.plot(in6_test_3_time_s, in6_test_3_amag_mps2, c='r', label = 'Test 3')
plt.xlabel('time (s)')
plt.ylabel('acceleration (mps2)')
plt.legend()
plt.show()

plt.title('0.96kg Drone Dropped from 0.3048m')
plt.plot(ft1_test_1_time_s, ft1_test_1_amag_mps2, c='b', label = 'Test 1')
plt.plot(ft1_test_2_time_s, ft1_test_2_amag_mps2, c='g', label = 'Test 2')
plt.plot(ft1_test_3_time_s, ft1_test_3_amag_mps2, c='r', label = 'Test 3')
plt.xlabel('time (s)')
plt.ylabel('acceleration (mps2)')
plt.legend()
plt.show()

plt.title('0.96kg Drone Dropped from 0.3048m')
plt.plot(ft2_test_1_time_s, ft2_test_1_amag_mps2, c='b', label = 'Test 1')
plt.plot(ft2_test_2_time_s, ft2_test_2_amag_mps2, c='g', label = 'Test 2')
plt.xlabel('time (s)')
plt.ylabel('acceleration (mps2)')
plt.legend()
plt.show()
