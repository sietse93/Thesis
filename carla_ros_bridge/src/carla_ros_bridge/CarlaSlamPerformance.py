#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import tf
import math


class CarlaSlamEvaluate(object):
    def __init__(self, method, flocation):
        self.method = method
        self.flocation = flocation
        self.time = []
        self.position = []
        self.orientation = []
        self.Q = []
        self.data = {}
        self.Q1Q2 = []
        self.timeQ1Q2 = []

    def __enter__(self):
        self.data = open(self.flocation, "r")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.data.close()
        return None

    def process_data(self):
        index = 0
        for line_data in self.data:
            if self.method == "gt":
                # Groundtruth is seperated by two spaces
                line = line_data.split("  ")
                floatLine = [float(element) for element in line[0:-1]]
                self.time.append(floatLine[0])
                # Make sure that vehicle starts at 0
                if index == 0:
                    init_pos = np.array(floatLine[1:4])
                    self.position.append(np.array(floatLine[1:4])-init_pos)
                else:
                    self.position.append(np.array(floatLine[1:4])-init_pos)
                quaternions = (floatLine[4:8])
                self.orientation.append(tf.transformations.euler_from_quaternion(quaternions))  # roll, pitch, yaw
                q = tf.transformations.quaternion_matrix(quaternions)
                q[0][3] = floatLine[1]
                q[1][3] = floatLine[2]
                q[2][3] = floatLine[3]
                self.Q.append(q)
                index = index + 1

            if self.method == "orb":
                line = line_data.split(" ")
                floatLine = [float(element) for element in line]
                self.time.append(floatLine[0])
                # Convert orb axis to ros axis
                new_position = [floatLine[3], -floatLine[1], -floatLine[2]]
                self.position.append(new_position)
                quaternions = floatLine[4:8]
                orb_roll, orb_pitch, orb_yaw = tf.transformations.euler_from_quaternion(quaternions)
                new_orientation = [orb_yaw, -orb_roll, -orb_pitch]
                self.orientation.append(new_orientation)
                quaternions = tf.transformations.quaternion_from_euler(new_orientation[0], new_orientation[1], new_orientation[2])
                q = tf.transformations.quaternion_matrix(quaternions)
                q[0][3] = new_position[0]
                q[1][3] = new_position[1]
                q[2][3] = new_position[2]
                self.Q.append(q)


def evaluate_trajectory(gt=CarlaSlamEvaluate, Slam = CarlaSlamEvaluate):
    # extract data
    gt_x = [positions[0] for positions in gt.position]
    gt_y = [positions[1] for positions in gt.position]
    Slam_x = [positions[0] for positions in Slam.position]
    Slam_y = [positions[1] for positions in Slam.position]

    plt.figure("Trajectory")
    plt.title('Trajectory')
    plt.xlabel("x position")
    plt.ylabel("y position")
    plt.plot(gt_x, gt_y, label=gt.method)
    plt.plot(Slam_x, Slam_y, label=Slam.method)
    plt.legend()


def evaluate_pose_over_time(gt=CarlaSlamEvaluate, Slam = CarlaSlamEvaluate):
    gt_x = [positions[0] for positions in gt.position]
    gt_y = [positions[1] for positions in gt.position]
    gt_z = [positions[2] for positions in gt.position]
    gt_roll = [orientations[0] for orientations in gt.orientation]
    gt_pitch = [orientations[1] for orientations in gt.orientation]
    gt_yaw = [orientations[2] for orientations in gt.orientation]

    Slam_x = [positions[0] for positions in Slam.position]
    Slam_y = [positions[1] for positions in Slam.position]
    Slam_z = [positions[2] for positions in Slam.position]
    Slam_roll = [orientations[0] for orientations in Slam.orientation]
    Slam_pitch = [orientations[1] for orientations in Slam.orientation]
    Slam_yaw = [orientations[2] for orientations in Slam.orientation]

    plt.figure("Pose over time")
    plt.title('Pose over time')

    plt.subplot(3, 2, 1)
    plt.plot(gt.time, gt_x, label=gt.method)
    plt.plot(Slam.time, Slam_x, label=Slam.method)
    plt.xlabel("time [s]")
    plt.ylabel("x position")

    plt.subplot(3, 2, 3)
    plt.plot(gt.time, gt_y, label=gt.method)
    plt.plot(Slam.time, Slam_y, label=Slam.method)
    plt.xlabel("time [s]")
    plt.ylabel("y position")

    plt.subplot(3, 2, 5)
    plt.plot(gt.time, gt_z, label=gt.method)
    plt.plot(Slam.time, Slam_z, label=Slam.method)
    plt.xlabel("time [s]")
    plt.ylabel("z position")

    plt.subplot(3, 2, 2)
    plt.plot(gt.time, gt_roll, label=gt.method)
    plt.plot(Slam.time, Slam_roll, label=Slam.method)
    plt.xlabel("time [s]")
    plt.ylabel("roll [rad]")

    plt.subplot(3, 2, 4)
    plt.plot(gt.time, gt_pitch, label=gt.method)
    plt.plot(Slam.time, Slam_pitch, label=Slam.method)
    plt.xlabel("time [s]")
    plt.ylabel("pitch [rad]")

    plt.subplot(3, 2, 6)
    plt.plot(gt.time, gt_yaw, label=gt.method)
    plt.plot(Slam.time, Slam_yaw, label=Slam.method)
    plt.xlabel("time [s]")
    plt.ylabel("yaw [rad]")

    plt.legend()


def evaluate_PSE(gt = CarlaSlamEvaluate, Slam = CarlaSlamEvaluate):
    RPE = []

    for time in Slam.time:
        if time > 2 and time < (Slam.time[-1]-1):
            Q1 = Slam.Q[Slam.time.index(time)]
            Q1_inv = np.linalg.inv(Q1)

            try:
                Q2 = Slam.Q[Slam.time.index(time+1)]
                Q1Q2_i = Q1_inv.dot(Q2)
                Slam.Q1Q2.append(Q1Q2_i)
                Slam.timeQ1Q2.append(time)

                # get equivalent gt time index

                gt_index = gt.time.index(time)
                Q1_gt = gt.Q[gt_index]
                Q1_gt_inv = np.linalg.inv(Q1_gt)
                Q2_gt = gt.Q[gt.time.index(time+1)]
                Q1Q2_gt_i = Q1_gt_inv.dot(Q2_gt)
                gt.Q1Q2.append(Q1Q2_gt_i)
                gt.timeQ1Q2.append(time)
                Q1Q2_gt_i_inv = np.linalg.inv(Q1Q2_gt_i)
                RPE_i = Q1Q2_gt_i_inv.dot(Q1Q2_i)
                RPE.append(RPE_i)

            except ValueError:
                pass

    RPEx = [matrix[0][3] for matrix in RPE]
    RPEy = [matrix[1][3] for matrix in RPE]
    RPEz = [matrix[2][3] for matrix in RPE]

    Q1Q2x = [matrix[0][3] for matrix in Slam.Q1Q2]
    Q1Q2y = [matrix[1][3] for matrix in Slam.Q1Q2]
    Q1Q2z = [matrix[2][3] for matrix in Slam.Q1Q2]

    Q1Q2gtx = [matrix[0][3] for matrix in gt.Q1Q2]
    Q1Q2gty = [matrix[1][3] for matrix in gt.Q1Q2]
    Q1Q2gtz = [matrix[2][3] for matrix in gt.Q1Q2]

    plt.figure("Q1Q2")
    plt.subplot(3, 1, 1)
    plt.plot(gt.timeQ1Q2, Q1Q2gtx, label=gt.method)
    plt.plot(Slam.timeQ1Q2, Q1Q2x, label=Slam.method)

    plt.subplot(3, 1, 2)
    plt.plot(gt.timeQ1Q2, Q1Q2gty, label=gt.method)
    plt.plot(Slam.timeQ1Q2, Q1Q2y, label=Slam.method)

    plt.subplot(3, 1, 3)
    plt.plot(gt.timeQ1Q2, Q1Q2gtz, label=gt.method)
    plt.plot(Slam.timeQ1Q2, Q1Q2z, label=Slam.method)


    plt.figure("RPE")
    plt.subplot(3, 1, 1)
    plt.plot(Slam.timeQ1Q2, RPEx)
    plt.subplot(3, 1, 2)
    plt.plot(Slam.timeQ1Q2, RPEy)
    plt.subplot(3, 1, 3)
    plt.plot(Slam.timeQ1Q2, RPEz)


def main():
    method_gt = "gt"
    gt_file= "/home/sietse/carla_experiment_data/stereo_static_short.txt"
    with CarlaSlamEvaluate(method_gt, gt_file) as gt_data:
        gt_data.process_data()

    method_orb = "orb"
    orb_file = "/home/sietse/carla_experiment_data/stereo_static_short_orb.txt"
    with CarlaSlamEvaluate(method_orb, orb_file) as orb_data:
        orb_data.process_data()

    evaluate_trajectory(gt=gt_data, Slam=orb_data)

    evaluate_pose_over_time(gt=gt_data, Slam=orb_data)

    evaluate_PSE(gt=gt_data, Slam=orb_data)

    plt.show()


if __name__=="__main__":
    main()
