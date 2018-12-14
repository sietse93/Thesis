#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import tf
from pyquaternion import Quaternion


class CarlaSlamEvaluate(object):
    def __init__(self, method, flocation):

        # contains a method which will define how the data is processed
        self.method = method

        # .txt file location
        self.flocation = flocation

        # label used for plotting
        self.label = ""

        # game timestamp in seconds (float)
        self.time = []

        # position [x, y, z] initialised at [0, 0, 0]
        # Use a right handed coordinate system (z points upwards)
        self.position = []

        # orientation [roll, pitch, yaw] in degrees
        self.orientations = []

        # orientation expressed in quaternions
        self.quaternions = []

        # Homogeneous coordinate matrix
        self.Q = []

        # Contains the data used for the open() function
        self.data = {}

        # Relative pose change over a certain time frame expressed in homogeneous coordinates
        self.Q1Q2 = []

        # A debugging property that logs the time_stamps used for linear interpolation.
        self.timeQ1Q2 = []

    def __enter__(self):
        self.data = open(self.flocation, "r")
        # This should label the object to the file name which should be descriptive enough
        # Removes "/home/sietse/carla_experiment" from string (35 char long)
        self.label = self.flocation[35:]
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.data.close()
        return None

    def process_data(self):
        """ Makes an object of .txt pose data """

        # index is to determine if this is the first datapoint
        index = 0
        for line_data in self.data:
            if self.method == "gt":
                # Groundtruth is seperated by a space
                line = line_data.split(" ")
                float_line = [float(element) for element in line]

                # convert time from ms to s
                time = float_line[0]*10**(-3)
                self.time.append(time)

                # position [x, y, z], but a left handed coordinate system
                # we convert the axis system to a ros coordinate system, which is right handed
                # note: orientation is in degrees
                x = float_line[1]
                y = - float_line[2]
                z = float_line[3]
                roll = -float_line[4]
                pitch = float_line[5]
                yaw = - float_line[6]
                position = np.array([x, y, z])
                orientation = np.array([roll, pitch, yaw])

                # Make sure that vehicle starts at 0
                if index == 0:
                    init_pos = position
                self.position.append(position-init_pos)
                self.orientations.append(orientation)

                # convert euler angles to quaternions
                # rotation order: static axis, roll, pitch, yaw
                quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
                self.quaternions.append(quaternion)

                # Convert quaternion to homogeneous coordinates
                # Note: NEVER GO FROM ROTATION MATRIX TO QUATERNIONS
                # One rotation matrix has 2 quaternion values
                q = tf.transformations.quaternion_matrix(quaternion)
                q[0][3] = x
                q[1][3] = y
                q[2][3] = z
                self.Q.append(q)
                index = index + 1

            if self.method == "orb":
                line = line_data.split(" ")
                floatLine = [float(element) for element in line]
                self.time.append(floatLine[0])
                # Convert orb axis to ros axis
                new_position = [-floatLine[3], floatLine[1], -floatLine[2]]
                self.position.append(new_position)
                quaternion = floatLine[4:8]
                # orb_roll, orb_pitch, orb_yaw = tf.transformations.euler_from_quaternion(quaternions)
                # new_orientation = [orb_yaw, -orb_roll, -orb_pitch]
                # self.orientation.append(new_orientation)
                # quaternions = tf.transformations.quaternion_from_euler(new_orientation[0], new_orientation[1], new_orientation[2])
                self.quaternions.append(quaternion)
                q = tf.transformations.quaternion_matrix(quaternion)
                q[0][3] = new_position[0]
                q[1][3] = new_position[1]
                q[2][3] = new_position[2]
                self.Q.append(q)


def compare_quaternions(methods):
    """Plots the quaternions of a list of CarlaSlamEvaluate object """
    plt.figure("Quaternions")

    for method in methods:
        q1 = [quaternion[0] for quaternion in method.quaternions]
        q2 = [quaternion[1] for quaternion in method.quaternions]
        q3 = [quaternion[2] for quaternion in method.quaternions]
        q4 = [quaternion[3] for quaternion in method.quaternions]

        plt.subplot(4, 1, 1)
        plt.plot(method.time, q1, label=method.label)
        plt.subplot(4, 1, 2)
        plt.plot(method.time, q2, label=method.label)
        plt.subplot(4, 1, 3)
        plt.plot(method.time, q3, label=method.label)
        plt.subplot(4, 1, 4)
        plt.plot(method.time, q4, label=method.label)

    plt.legend()


def compare_euler_angles(methods):
    plt.figure("Euler_angles")

    plt.subplot(3, 1, 1)
    plt.xlabel("time [s]")
    plt.ylabel("roll [deg]")

    plt.subplot(3, 1, 2)
    plt.xlabel("time [s]")
    plt.ylabel("pitch [deg]")

    plt.subplot(3, 1, 3)
    plt.xlabel("time [s]")
    plt.ylabel("yaw [deg]")

    for method in methods:
        roll = [orientation[0] for orientation in method.orientations]
        pitch = [orientation[1] for orientation in method.orientations]
        yaw = [orientation[2] for orientation in method.orientations]

        plt.subplot(3, 1, 1)
        plt.plot(method.time, roll, label=method.label)

        plt.subplot(3, 1, 2)
        plt.plot(method.time, pitch, label=method.label)

        plt.subplot(3, 1, 3)
        plt.plot(method.time, yaw, label=method.label)

    plt.legend()


def evaluate_trajectory(GT, SLAM):

    plt.figure("Trajectory")
    plt.title('Trajectory')
    plt.xlabel("x position")
    plt.ylabel("y position")

    for gt in GT:
        gt_x = [positions[0] for positions in gt.position]
        gt_y = [positions[1] for positions in gt.position]
        plt.plot(gt_x, gt_y, label=gt.label)

    for Slam in SLAM:
        Slam_x = [positions[0] for positions in Slam.position]
        Slam_y = [positions[1] for positions in Slam.position]
        plt.plot(Slam_x, Slam_y, label=Slam.label)

    plt.legend()


def evaluate_pose_over_time(GT, SLAM):

    """Plots the pose (xyz and Euler angles) over time of both the groundtruth and the SLAM algorithm"""

    # create layout for the plots
    # plt.figure("Pose over time")
    plt.figure()
    plt.title('Pose over time')

    plt.subplot(3, 2, 1)
    plt.xlabel("time [s]")
    plt.ylabel("x position")

    plt.subplot(3, 2, 3)
    plt.xlabel("time [s]")
    plt.ylabel("y position")

    plt.subplot(3, 2, 5)
    plt.xlabel("time [s]")
    plt.ylabel("z position")

    plt.subplot(3, 2, 2)
    plt.xlabel("time [s]")
    plt.ylabel("roll [rad]")

    plt.subplot(3, 2, 4)
    plt.xlabel("time [s]")
    plt.ylabel("pitch [rad]")

    plt.subplot(3, 2, 6)
    plt.xlabel("time [s]")
    plt.ylabel("yaw [rad]")

    # plot all the groundtruths
    for gt in GT:
        gt_x = [positions[0] for positions in gt.position]
        gt_y = [positions[1] for positions in gt.position]
        gt_z = [positions[2] for positions in gt.position]
        gt_roll = [orientations[0] for orientations in gt.orientation]
        gt_pitch = [orientations[1] for orientations in gt.orientation]
        gt_yaw = [orientations[2] for orientations in gt.orientation]

        plt.subplot(3, 2, 1)
        plt.plot(gt.time, gt_x, label=gt.label)

        plt.subplot(3, 2, 3)
        plt.plot(gt.time, gt_y, label=gt.label)

        plt.subplot(3, 2, 5)
        plt.plot(gt.time, gt_z, label=gt.label)

        plt.subplot(3, 2, 2)
        plt.plot(gt.time, gt_roll, label=gt.label)

        plt.subplot(3, 2, 4)
        plt.plot(gt.time, gt_pitch, label=gt.label)

        plt.subplot(3, 2, 6)
        plt.plot(gt.time, gt_yaw, label=gt.label)

    # plot all the SLAM data
    for Slam in SLAM:
        Slam_x = [positions[0] for positions in Slam.position]
        Slam_y = [positions[1] for positions in Slam.position]
        Slam_z = [positions[2] for positions in Slam.position]
        Slam_roll = [orientations[0] for orientations in Slam.orientation]
        Slam_pitch = [orientations[1] for orientations in Slam.orientation]
        Slam_yaw = [orientations[2] for orientations in Slam.orientation]

        plt.subplot(3, 2, 1)
        plt.plot(Slam.time, Slam_x, label=Slam.label)

        plt.subplot(3, 2, 3)
        plt.plot(Slam.time, Slam_y, label=Slam.label)

        plt.subplot(3, 2, 5)
        plt.plot(Slam.time, Slam_z, label=Slam.label)

        plt.subplot(3, 2, 2)
        plt.plot(Slam.time, Slam_roll, label=Slam.label)

        plt.subplot(3, 2, 4)
        plt.plot(Slam.time, Slam_pitch, label=Slam.label)

        plt.subplot(3, 2, 6)
        plt.plot(Slam.time, Slam_yaw, label=Slam.label)

    plt.legend()


def evaluate_PSE(gt=CarlaSlamEvaluate, Slam=CarlaSlamEvaluate, time_step=float):
    RPE = []
    time_debug = []
    Q1Q2_debug = []

    for time in Slam.time:
        if time > 2 and time < (Slam.time[-1]-time_step):
            time_index = Slam.time.index(time)
            Q1 = Slam.Q[time_index]
            Q1_inv = np.linalg.inv(Q1)

            try:
                Q2 = Slam.Q[Slam.time.index(time+time_step)]  # add time_step if it doesnt exist it gets a ValueError

            except ValueError:
                # if time step does not work, find the closest timestamp that is bigger than the time step
                temp_index = time_index+1
                time2 = Slam.time[temp_index]
                while time2 < time + 1:
                    temp_index = temp_index + 1
                    time2 = Slam.time[temp_index]

                #  use this data to linear interpolate the position
                position1 = np.asarray(Slam.position[time_index])
                position2 = np.asarray(Slam.position[temp_index])
                position_inter = time_step*(position2 - position1)/(time2-time)+position1

                # use Slerp technique to linear interpolate the rotations
                q1 = Quaternion(Slam.quaternion[time_index])
                q2 = Quaternion(Slam.quaternion[temp_index])
                time_inter = time_step/(time2-time)
                q = Quaternion.slerp(q1, q2, time_inter)
                q_inter = [q[0], q[1], q[2], q[3]]
                Q2 = tf.transformations.quaternion_matrix(q_inter)
                Q2[0][3] = position_inter[0]
                Q2[1][3] = position_inter[1]
                Q2[2][3] = position_inter[2]

                # some debug variables that allows you to visualize the linear interpolated data
                time_debug.append(time)
                Q1Q2_debug.append(Q1_inv.dot(Q2))

            Q1Q2_i = Q1_inv.dot(Q2)
            Slam.Q1Q2.append(Q1Q2_i)
            Slam.timeQ1Q2.append(time)

            # get equivalent gt time index

            gt_index = gt.time.index(time)
            Q1_gt = gt.Q[gt_index]
            Q1_gt_inv = np.linalg.inv(Q1_gt)
            Q2_gt = gt.Q[gt.time.index(time + time_step)]
            Q1Q2_gt_i = Q1_gt_inv.dot(Q2_gt)
            gt.Q1Q2.append(Q1Q2_gt_i)
            gt.timeQ1Q2.append(time)
            Q1Q2_gt_i_inv = np.linalg.inv(Q1Q2_gt_i)
            RPE_i = Q1Q2_gt_i_inv.dot(Q1Q2_i)
            print(RPE_i)
            RPE.append(RPE_i)

    RPEx = [matrix[0][3] for matrix in RPE]
    RPEy = [matrix[1][3] for matrix in RPE]
    RPEz = [matrix[2][3] for matrix in RPE]

    Q1Q2x = [matrix[0][3] for matrix in Slam.Q1Q2]
    Q1Q2y = [matrix[1][3] for matrix in Slam.Q1Q2]
    Q1Q2z = [matrix[2][3] for matrix in Slam.Q1Q2]

    Q1Q2gtx = [matrix[0][3] for matrix in gt.Q1Q2]
    Q1Q2gty = [matrix[1][3] for matrix in gt.Q1Q2]
    Q1Q2gtz = [matrix[2][3] for matrix in gt.Q1Q2]

    Q1Q2_debugx = [matrix[0][3] for matrix in Q1Q2_debug]
    Q1Q2_debugy = [matrix[1][3] for matrix in Q1Q2_debug]
    Q1Q2_debugz = [matrix[2][3] for matrix in Q1Q2_debug]

    plt.figure("Q1Q2")
    plt.subplot(3, 1, 1)
    plt.plot(gt.timeQ1Q2, Q1Q2gtx, label=gt.method)
    plt.plot(Slam.timeQ1Q2, Q1Q2x, label=Slam.method)
    # plt.plot(time_debug, Q1Q2_debugx, 'o', label='linear interpolated')

    plt.subplot(3, 1, 2)
    plt.plot(gt.timeQ1Q2, Q1Q2gty, label=gt.method)
    plt.plot(Slam.timeQ1Q2, Q1Q2y, label=Slam.method)
    # plt.plot(time_debug, Q1Q2_debugy, 'o', label='linear interpolated')

    plt.subplot(3, 1, 3)
    plt.plot(gt.timeQ1Q2, Q1Q2gtz, label=gt.method)
    plt.plot(Slam.timeQ1Q2, Q1Q2z, label=Slam.method)
    # plt.plot(time_debug, Q1Q2_debugz, 'o', label='linear interpolated')

    plt.figure("RPE")
    plt.subplot(3, 1, 1)
    plt.plot(Slam.timeQ1Q2, RPEx)
    plt.subplot(3, 1, 2)
    plt.plot(Slam.timeQ1Q2, RPEy)
    plt.subplot(3, 1, 3)
    plt.plot(Slam.timeQ1Q2, RPEz)


def main():

    method_gt = "gt"
    gt_file = "/home/sietse/NV_30_SV_1_gt.txt"
    with CarlaSlamEvaluate(method_gt, gt_file) as gt_data:
        gt_data.process_data()

    compare_quaternions([gt_data])
    compare_euler_angles([gt_data])
    plt.show()


if __name__=="__main__":
    main()
