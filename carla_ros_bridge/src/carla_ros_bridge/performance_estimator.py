#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import tf
import math

gt_file = open("/home/sietse/carla_experiment_data/stereo_static_short.txt", "r")

time_stamp_gt = []
Q_gt = []
x_gt = []
y_gt = []
z_gt = []

for log_line in gt_file:
    temp = log_line.split("  ")
    time_stamp_gt.append(float(temp[0]))
    x = float(temp[1])
    y = float(temp[2])
    z = float(temp[3])
    quaternions = np.array(temp[4:-1])
    Q = tf.transformations.quaternion_matrix(quaternions)
    x_gt.append(x)
    y_gt.append(y)
    z_gt.append(z)
    Q[0][3] = x
    Q[1][3] = y
    Q[2][3] = z
    Q_gt.append(Q)
gt_file.close()

x_gt = np.asarray(x_gt) - x_gt[0]
y_gt = np.asarray(y_gt) - y_gt[0]
z_gt = np.asarray(z_gt) - z_gt[0]


orb_file = open("/home/sietse/carla_experiment_data/stereo_static_short_orb.txt", "r")

time_stamp_orb = []
Q_orb = []
x_orb = []
y_orb = []
z_orb = []

for log_line in orb_file:
    temp = log_line.split(" ")
    time_stamp_orb.append(float(temp[0]))
    x = float(temp[3])
    y = -float(temp[1])
    z = float(temp[2])
    # x = float(temp[1])
    # y = float(temp[2])
    # z = float(temp[3])
    q1 = float(temp[4])
    q2 = float(temp[5])
    q3 = float(temp[6])
    q4 = float(temp[7])
    quaternions = [q1, q2, q3, q4]
    Q = tf.transformations.quaternion_matrix(quaternions)
    x_orb.append(x)
    y_orb.append(y)
    z_orb.append(z)
    Q[0][3] = x
    Q[1][3] = y
    Q[2][3] = z
    Q_orb.append(Q)

orb_file.close()

x_orb = np.asarray(x_orb)
y_orb = np.asarray(y_orb)
z_orb = np.asarray(z_orb)

# Walk through all the orb timestamps, calculate relative pose change
# start from 2 seconds onward, algorithm not stable before
# Add 1 second to the timestamp and find the closest timestamp.
# If it exists, calculate the relative pose change
# If there does not exist a timeslot interpolate a value for that timestamp
Q_diff_orb = []
x_diff_orb = []
y_diff_orb = []
z_diff_orb = []
time_used = []
Q_diff_gt = []
x_diff_gt = []
y_diff_gt = []
z_diff_gt = []

# Debugging interpolation data
# time_inter = []
# Q_inter = []
# x_inter = []
# y_inter = []
# z_inter = []

# in plane translation
trans_gt = []
trans_orb = []
for time in time_stamp_orb:
    if time > 2 and time < (time_stamp_orb[-1]-1):
        Q1_orb = Q_orb[time_stamp_orb.index(time)]
        Q1_inv_orb = np.linalg.inv(Q1_orb)

        try:
            Q2_orb = Q_orb[time_stamp_orb.index(time+1)]
            Q_DiffOrb = Q1_inv_orb.dot(Q2_orb)
            Q_diff_orb.append(Q_DiffOrb)
            x_diff_orb.append(Q_DiffOrb[0][3])
            y_diff_orb.append(Q_DiffOrb[1][3])
            z_diff_orb.append(Q_DiffOrb[2][3])
            trans_orb.append(math.sqrt(Q_DiffOrb[0][3] ** 2 + Q_DiffOrb[1][3] ** 2))
            time_used.append(time)

            # Get the equivalent ground truth index
            gt_index = time_stamp_gt.index(time)
            Q1_gt = Q_gt[gt_index]
            Q2_gt = Q_gt[time_stamp_gt.index(time + 1)]
            # print(Q1_gt)
            Q1_gt_inv = np.linalg.inv(Q1_gt)
            Q_DiffGt = Q1_gt_inv.dot(Q2_gt)
            Q_diff_gt.append(Q_DiffGt)

            x_diff_gt.append(Q_DiffGt[0][3])
            y_diff_gt.append(Q_DiffGt[1][3])
            z_diff_gt.append(Q_DiffGt[2][3])
            trans_gt.append(math.sqrt(Q_DiffGt[0][3] ** 2 + Q_DiffGt[1][3] ** 2))
        except ValueError:
            pass
            # find the next value that is closest to the one second mark
            # data is linearly interpolated

            # temp_index = time_stamp_orb.index(time)
            # while time_stamp_orb[temp_index] < time+1:
            #     temp_index = temp_index+1
            # Q2_orb = Q1_orb+(time_stamp_orb[temp_index]-time)/time_stamp_orb[temp_index]*(Q_orb[temp_index]-Q1_orb)
            # Q_DiffOrb = Q1_inv_orb.dot(Q2_orb)
            # Q_diff_orb.append(Q_DiffOrb)
            # time_inter.append(time)
            # Q_inter.append(Q_DiffOrb)
            # x_inter.append(Q_DiffOrb[0][3])
            # y_inter.append(Q_DiffOrb[1][3])
            # z_inter.append(Q_DiffOrb[2][3])

        # x_diff_orb.append(Q_DiffOrb[0][3])
        # y_diff_orb.append(Q_DiffOrb[1][3])
        # z_diff_orb.append(Q_DiffOrb[2][3])
        # trans_orb.append(math.sqrt(Q_DiffOrb[0][3]**2+Q_DiffOrb[1][3]**2))

        # # Get the equivalent ground truth index
        # gt_index = time_stamp_gt.index(time)
        # Q1_gt = Q_gt[gt_index]
        # Q2_gt = Q_gt[time_stamp_gt.index(time + 1)]
        # # print(Q1_gt)
        # Q1_gt_inv = np.linalg.inv(Q1_gt)
        # Q_DiffGt = Q1_gt_inv.dot(Q2_gt)
        # Q_diff_gt.append(Q_DiffGt)
        #
        # x_diff_gt.append(Q_DiffGt[0][3])
        # y_diff_gt.append(Q_DiffGt[1][3])
        # z_diff_gt.append(Q_DiffGt[2][3])
        # trans_gt.append(math.sqrt(Q_DiffGt[0][3]**2 + Q_DiffGt[1][3]**2))

# data analysis

# check if the linear interpolation went wel. Check if there are abnormal spikes in the data
plt.figure(1)
plt.title('Groundtruth Trajectory')
plt.plot(x_gt, y_gt)
plt.xlabel("x groundtruth")
plt.ylabel("y groundtruth")

# groundtruth actual x location
plt.figure(2)
plt.subplot(3, 1, 1)
plt.plot(time_stamp_gt, x_gt, label='absolute x location gt')
plt.plot(time_stamp_orb, x_orb, label='absolute x location orb')
plt.xlabel("time")
plt.ylabel("x location")

plt.subplot(3, 1, 2)
plt.plot(time_stamp_gt, y_gt, label='absolute y location gt')
plt.plot(time_stamp_orb, y_orb, label='absolute y location orb')
plt.xlabel("time")
plt.ylabel("y location")


plt.subplot(3, 1, 3)
plt.plot(time_stamp_gt, z_gt, label='absolute z location gt')
plt.plot(time_stamp_orb, z_orb, label='absolute z location orb')
plt.xlabel("time")
plt.ylabel("z location")
plt.legend()

plt.figure(3)
plt.subplot(3, 1, 1)
plt.plot(time_used, x_diff_gt, '-o', label='x difference gt')
plt.plot(time_used, x_diff_orb, '-o', label='x difference orb')
# plt.plot(time_inter, x_inter, 'o', label='interpolation points')
plt.xlabel("time")
plt.ylabel("relative difference x")

plt.subplot(3, 1, 2)
plt.plot(time_used, y_diff_gt, '-o', label='y difference gt')
plt.plot(time_used, y_diff_orb, '-o',label='y difference orb')
# plt.plot(time_inter, y_inter, 'o', label='interpolation points')
plt.xlabel("time")
plt.ylabel("relative difference y")

plt.subplot(3, 1, 3)
plt.plot(time_used, z_diff_gt, '-o', label='z difference gt')
plt.plot(time_used, z_diff_orb, '-o', label='z difference orb')
# plt.plot(time_inter, z_inter, 'o', label='interpolation points')
plt.xlabel("time")
plt.ylabel("relative difference z")
plt.legend()

plt.figure(4)
plt.plot(time_used, trans_gt, label='planar velocity gt')
plt.plot(time_used, trans_orb, label='planar velocity orb')
plt.xlabel('time')
plt.ylabel('change in magnitude translation')

plt.show()
