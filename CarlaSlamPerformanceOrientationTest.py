#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import tf
from pyquaternion import Quaternion
import math
import pdb

class CarlaSlamEvaluate(object):
    def __init__(self, method, flocation, plotstyle):

        # contains a method which will define how the data is processed
        self.method = method

        # .txt file location
        self.flocation = flocation

        # label used for plotting
        self.label = ""

        # defines how the method is presented in a graph
        self.plotstyle = plotstyle

        # game timestamp in seconds (float)
        self.time = []

        # position [x, y, z] initialised at [0, 0, 0]
        # Use a right handed coordinate system (z points upwards)
        self.positions = []

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

        # time stamps used for RPE
        self.timeQ1Q2 = []

        # Root Mean Square Error of Relative Pose Error over distance
        self.RPE_RMSE_dist = []


    def __enter__(self):
        self.data = open(self.flocation, "r")
        # This should label the object to the file name which should be descriptive enough
        # index where "SL" starts
        index_start = self.flocation.find("SL")

        # label = NameOfMethod_SL_{}_NV_{}
        # self.label = self.method + "_" + self.flocation[index_start:(index_start+11)]
        self.label = self.method + "_" + self.flocation[index_start:-4]
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.data.close()
        return None

    def process_data(self):
        """ Makes an object of .txt pose data """

        if self.method == "gt":

            # pose variables that come from the groundtruth txt file
            x_ue_abs = []
            y_ue_abs = []
            z_ue_abs = []
            roll_ue_abs = []
            pitch_ue_abs = []
            yaw_ue_abs180 = []
            for line_data in self.data:
                # Groundtruth is seperated by a space
                line = line_data.split(" ")
                float_line = [float(element) for element in line]

                # convert time from [ms] to [s]
                # time = round(float_line[0]*10**(-3), 4)
                time = float_line[0]
                self.time.append(time)

                # groundtruth is absolute position in unreal engine coordinate system
                # left handed coordinate system
                # orientation is in degrees
                x_ue_abs.append(float_line[1])
                y_ue_abs.append(float_line[2])
                z_ue_abs.append(float_line[3])
                roll_ue_abs.append(float_line[4])
                pitch_ue_abs.append(float_line[5])

                # note: this yaw angle is between -180 and 180 in a left handed system
                yaw_ue_abs180.append(float_line[6])

            # convert the yaw angle to an angle that goes beyond 180 degrees
            yaw_ue_abs = abs_yaw_angle_conversion(yaw_ue_abs180)

            # Input are the coordinates of the vehicle in an absolute left handed system, specified by UE
            # Yaw angle is the absolute rotation measured from the UE axis system, which allows to specify full rotation
            for index, time in enumerate(self.time):
                # Make sure that vehicle starts at 0
                if index == 0:
                    x_ue_init = x_ue_abs[index]
                    y_ue_init = y_ue_abs[index]
                    z_ue_init = z_ue_abs[index]
                    yaw_ue_init = yaw_ue_abs[index]

                # Make the pose relative to its starting position
                x_ue_0 = x_ue_abs[index] - x_ue_init
                y_ue_0 = y_ue_abs[index] - y_ue_init
                z_ue_0 = z_ue_abs[index] - z_ue_init
                yaw_ue_0 = yaw_ue_abs[index] - yaw_ue_init

                # Nothing changes with the Euler angles, except yaw.
                roll_ue_rel = roll_ue_abs[index]
                pitch_ue_rel = pitch_ue_abs[index]

                # new Carla can output a pitch of 360 degrees, which should be 0 degrees
                if pitch_ue_rel >= 360.0:
                    pitch_ue_rel = pitch_ue_rel - 360.0
                # yaw needs to be relative to initial position
                yaw_ue_rel = yaw_ue_0

                # convert absolute left handed system into a relative left handed system
                theta = np.radians(yaw_ue_init)
                c, s = np.cos(theta), np.sin(theta)
                # note this is a rotation matrix for LEFT HANDED COORDINATE SYSTEM
                R = np.matrix([[c, s, 0], [-s, c, 0], [0, 0, 1]])
                pos_ue_0 = np.matrix([x_ue_0, y_ue_0, z_ue_0])
                pos_ue_rel = R*pos_ue_0.reshape(3, 1)
                # matrix to floats
                x_ue_rel = pos_ue_rel.tolist()[0][0]
                y_ue_rel = pos_ue_rel.tolist()[1][0]
                z_ue_rel = pos_ue_rel.tolist()[2][0]

                # yaw_start = int(round(yaw_ue_init))
                # if yaw_start == 0:
                #     x_ue_rel = x_ue_0
                #     y_ue_rel = y_ue_0
                #     z_ue_rel = z_ue_0
                # elif yaw_start == 180:
                #     x_ue_rel = -x_ue_0
                #     y_ue_rel = -y_ue_0
                #     z_ue_rel = z_ue_0
                # elif yaw_start == 90:
                #     x_ue_rel = y_ue_0
                #     y_ue_rel = -x_ue_0
                #     z_ue_rel = z_ue_0
                # elif yaw_start == -90:
                #     x_ue_rel = - y_ue_0
                #     y_ue_rel = x_ue_0
                #     z_ue_rel = z_ue_0
                # else:
                #     print("Starting point is not along one of the axis")
                #     exit()

                # Convert relative left handed system to the right handed system used in ROS
                x = x_ue_rel
                y = - y_ue_rel
                z = z_ue_rel
                roll = -roll_ue_rel
                pitch = pitch_ue_rel
                yaw = - yaw_ue_rel

                position = np.array([x, y, z])
                orientation = np.array([roll, pitch, yaw])
                self.positions.append(position)
                self.orientations.append(orientation)

                # convert euler angles to quaternions
                # rotation order: static axis, roll, pitch, yaw
                # could be that there are change in sign in these quaternions
                quaternion = tf.transformations.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw), axes='sxyz')
                self.quaternions.append(quaternion)

                # Convert quaternion to homogeneous coordinates
                # Note: NEVER GO FROM ROTATION MATRIX TO QUATERNIONS
                # Since one rotation matrix has 2 quaternion values
                # q = tf.transformations.quaternion_matrix(quaternion)
                # q[0][3] = x
                # q[1][3] = y
                # q[2][3] = z

                # lets try to get the rotation matrix from the Euler angles
                q = tf.transformations.euler_matrix(math.radians(roll), math.radians(pitch), math.radians(yaw), axes='sxyz')
                q[0][3] = x
                q[1][3] = y
                q[2][3] = z
                self.Q.append(q)

        if self.method == "orb":

            # need to extract the Euler angles to convert yaw since it only indicates angle at +180, -180
            roll_list = []
            pitch_list = []
            yaw_180_list = []
            for line_data in self.data:
                line = line_data.split(" ")
                float_line = [float(element) for element in line]

                # time is already in [s]
                time = float_line[0]
                self.time.append(time)

                # pose estimation in ORB coordinate system
                x_orb = float_line[1]
                y_orb = float_line[2]
                z_orb = float_line[3]
                q1_orb = float_line[4]
                q2_orb = float_line[5]
                q3_orb = float_line[6]
                q4_orb = float_line[7]

                # Convert orb axis to ros axis, which in theory looks like
                # 1) CCW (+)90 deg in x-axis
                # 2) CCW (+)90 deg in z-axis

                # But it seems that positive rotations are defined as CW in the ORB system, instead of CCW.
                # That is the only reason how I can explain that x is correct and y and z are in the negative direction.
                # It is also not a left handed system, because the orientation of the axis are right handed.
                theta = np.radians(-90)
                c, s = np.cos(theta), np.sin(theta)
                # Right handed rotation matrices
                Rx = np.matrix([[1, 0, 0], [0, c, -s], [0, s, c]])
                Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])

                pos_old = np.matrix([x_orb, y_orb, z_orb])
                new_pos = Rz*Rx*pos_old.reshape(3, 1)
                x_new = new_pos.tolist()[0][0]
                y_new = new_pos.tolist()[1][0]
                z_new = new_pos.tolist()[2][0]
                # old method
                # x_new = z_orb
                # y_new = -x_orb
                # z_new = -y_orb

                position = np.array([x_new, y_new, z_new])
                self.positions.append(position)

                # Quaternions is defined as iq1+jq2+kq3+q4
                # The axis system of i, j, k  are incorrect and need to be converted to ROS axis system
                q_old = np.matrix([q1_orb, q2_orb, q3_orb])
                q_new = Rz*Rx*q_old.reshape(3, 1)
                # Now the quaternion axis system has the same orientation as the euclidean space,
                # since the ORB axis system underwent the same transformation
                q1_new = q_new.tolist()[0][0]
                q2_new = q_new.tolist()[1][0]
                q3_new = q_new.tolist()[2][0]
                q4_new = q4_orb

                quaternion = [q1_new, q2_new, q3_new, q4_new]
                self.quaternions.append(quaternion)
                q = tf.transformations.quaternion_matrix(quaternion)
                q[0][3] = x_new
                q[1][3] = y_new
                q[2][3] = z_new
                self.Q.append(q)

                # Note that euler_from_quaternion, is quaternion_matrix function and then euler_from_matrix function
                # Also, positive rotations seem to be defined as CW instead of CCW.
                roll, pitch, yaw_180 = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')

                # roll and pitch are exactly reversed. If results are weird, this could be the mistake
                roll_list.append(np.degrees(-roll))
                pitch_list.append(np.degrees(-pitch))
                yaw_180_list.append(np.degrees(yaw_180))

            # convert yaw angle to an angle that can show more than 180 degrees
            yaw_list = abs_yaw_angle_conversion(yaw_180_list)

            # append the converted euler angles into the orientation attribute
            for index, roll in enumerate(roll_list):
                orientation = np.array([roll, pitch_list[index], yaw_list[index]])
                self.orientations.append(orientation)




def abs_yaw_angle_conversion(rel_yaw_angle):
    """Function that converts a yaw angle that ranges from  -180 to 180 degrees, to a yaw angle that has infinite range
    and indicates full rotations. Note that this function can be used for left and right handed axis system"""

    yaw_abs = []
    yaw_abs.append(rel_yaw_angle[0])

    # n180 tracks the number of 180 degrees rotations and direction.
    n180 = 0

    # sign_modulo is the sign of the very first pose
    # scratch that... second pose, first pose the angle can be zero for ORB SLAM. The method does nto work then.
    sign_modulo = np.sign(rel_yaw_angle[1])

    index = 1
    index_limit = len(rel_yaw_angle)

    debug_yaw = open("/home/sietse/debug_yaw.txt", "w")
    while index != index_limit:
        # finds the modulo of either 180 or -180
        modulo_angle = rel_yaw_angle[index] % (sign_modulo*180)

        if myround(rel_yaw_angle[index - 1]) == -180 and myround(rel_yaw_angle[index]) == 180:
            n180 = n180 - 180

        if myround(rel_yaw_angle[index - 1]) == 180 and myround(rel_yaw_angle[index]) == -180:
            n180 = n180 + 180

        if myround(rel_yaw_angle[index]) == 0 and np.sign(rel_yaw_angle[index - 1]) == -1 and np.sign(rel_yaw_angle[index]) == 1:
            n180 = n180 + 180

        if myround(rel_yaw_angle[index]) == 0 and np.sign(rel_yaw_angle[index - 1]) == 1 and np.sign(rel_yaw_angle[index]) == -1:
            n180 = n180 - 180

        yaw_abs_element = n180 + modulo_angle
        yaw_abs.append(yaw_abs_element)
        debug_yaw.write("{}     {}      {}  \n".format(index, rel_yaw_angle[index], yaw_abs_element))
        index = index + 1

    debug_yaw.close()
    return yaw_abs


def myround(x, base=10):
    """rounds the number to the nearest base, in this case 10
 ORB has jumps that go from -175 to 175, which are not detected if the round() function is used"""
    return int(round(x/base))*base


def main():
    method_gt = "gt"
    gt_ps_static = 'k-'
    gt_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_gt.txt"
    with CarlaSlamEvaluate(method_gt, gt_file_static, gt_ps_static) as gt_static:
        gt_static.process_data()

    method_orb = "orb"
    orb_ps_static = 'g-'
    orb_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_orb.txt"

    with CarlaSlamEvaluate(method_orb, orb_file_static, orb_ps_static) as orb_static:
        orb_static.process_data()


if __name__=="__main__":
    main()
