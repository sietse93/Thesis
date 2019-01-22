#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import tf
from pyquaternion import Quaternion
import math


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

    def __enter__(self):
        self.data = open(self.flocation, "r")
        # This should label the object to the file name which should be descriptive enough
        # index where "SL" starts
        index_start = self.flocation.find("SL")

        # label = NameOfMethod_SL_{}_NV_{}
        self.label = self.method + "_" + self.flocation[index_start:(index_start+11)]
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
                time = round(float_line[0]*10**(-3), 4)
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
                yaw_ue_rel = yaw_ue_0

                # convert absolute left handed system into a relative left handed system
                # I should probably use rotation matrices for this
                yaw_start = int(round(yaw_ue_init))
                if yaw_start == 0:
                    x_ue_rel = x_ue_0
                    y_ue_rel = y_ue_0
                    z_ue_rel = z_ue_0
                elif yaw_start == 180:
                    x_ue_rel = -x_ue_0
                    y_ue_rel = -y_ue_0
                    z_ue_rel = z_ue_0
                elif yaw_start == 90:
                    x_ue_rel = y_ue_0
                    y_ue_rel = -x_ue_0
                    z_ue_rel = z_ue_0
                elif yaw_start == -90:
                    x_ue_rel = - y_ue_0
                    y_ue_rel = x_ue_0
                    z_ue_rel = z_ue_0
                else:
                    print("Starting point is not along one of the axis")
                    exit()

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

                # Convert orb axis to ros axis
                x_new = z_orb
                y_new = -x_orb
                z_new = -y_orb
                position = [x_new, y_new, z_new]
                self.positions.append(position)

                # Quaternions is defined as iq1+jq2+kq3+q4
                # The axis system of i, j, k  are incorrect and need to be converted to ROS axis system

                # What I thought would give good results
                # q1_new = q3_orb
                # q2_new = -q1_orb
                # q3_new = -q2_orb
                # q4_new = q4_orb

                # What actually gives good results, (before the yaw angle outputted full rotations)
                q1_new = -q3_orb
                q2_new = q1_orb
                q3_new = -q2_orb
                q4_new = q4_orb

                quaternion = [q1_new, q2_new, q3_new, q4_new]
                self.quaternions.append(quaternion)
                q = tf.transformations.quaternion_matrix(quaternion)
                q[0][3] = x_new
                q[1][3] = y_new
                q[2][3] = z_new
                self.Q.append(q)

                roll, pitch, yaw_180 = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
                roll_list.append(np.degrees(roll))
                pitch_list.append(np.degrees(pitch))
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


# rounds the number to the nearest base, in this case 10
# ORB has jumps that go from -175 to 175, which are not detected if the round() function is used
def myround(x, base=10):
    return int(round(x/base))*base



def compare_position(methods):
    """plots the position of a list of CarlaSlamEvaluate objects"""

    plt.figure("Pose over time")

    for method in methods:
        x = [position[0] for position in method.positions]
        y = [position[1] for position in method.positions]
        z = [position[2] for position in method.positions]

        plt.subplot(3, 1, 1)
        plt.plot(method.time, x, label=method.label)
        plt.xlabel("time [s]")
        plt.ylabel("x location [m]")

        plt.subplot(3, 1, 2)
        plt.plot(method.time, y, label=method.label)
        plt.xlabel("time [s]")
        plt.ylabel("y location [m]")

        plt.subplot(3, 1, 3)
        plt.plot(method.time, z, label=method.label)
        plt.xlabel("time [s]")
        plt.ylabel("z location [m]")
    plt.legend()


def compare_quaternions(methods):
    """Plots the quaternions of a list of CarlaSlamEvaluate objects """
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


def evaluate_trajectory(methods):

    plt.figure("Trajectory")
    plt.title('Trajectory')
    plt.xlabel("x position")
    plt.ylabel("y position")

    for method in methods:
        x = [position[0] for position in method.positions]
        y = [position[1] for position in method.positions]
        plt.plot(x, y, label=method.label)
    plt.plot(methods[0].positions[0][0], methods[0].positions[0][1], 'x', label="starting point")

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
    plt.ylabel("roll [deg]")

    plt.subplot(3, 2, 4)
    plt.xlabel("time [s]")
    plt.ylabel("pitch [deg]")

    plt.subplot(3, 2, 6)
    plt.xlabel("time [s]")
    plt.ylabel("yaw [deg]")

    # plot all the groundtruths
    for gt in GT:
        gt_x = [position[0] for position in gt.positions]
        gt_y = [position[1] for position in gt.positions]
        gt_z = [position[2] for position in gt.positions]
        gt_roll = [orientation[0] for orientation in gt.orientations]
        gt_pitch = [orientation[1] for orientation in gt.orientations]
        gt_yaw = [orientation[2] for orientation in gt.orientations]

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
        Slam_x = [position[0] for position in Slam.positions]
        Slam_y = [position[1] for position in Slam.positions]
        Slam_z = [position[2] for position in Slam.positions]
        Slam_roll = [orientation[0] for orientation in Slam.orientations]
        Slam_pitch = [orientation[1] for orientation in Slam.orientations]
        Slam_yaw = [orientation[2] for orientation in Slam.orientations]

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



def evaluate_RPE(GT, SLAM, time_step=float):
    """Input a list of CarlaSlamEvaluate ground truths and a list of CarlaSlamEvaluate SLAM pose estimations"""

    # Check if every method has a ground truth
    if len(GT) != len(SLAM):
        print("Not every SLAM method has a ground truth")
        return

    # Evaluate Relative Pose Error
    # loop through each Slam method with associated ground truth
    for gt, Slam in zip(GT, SLAM):
        RPE = []
        time_debug = []
        Q1Q2_debug = []

        trans_errs = []
        rot_errs = []

        for time in Slam.time:
            # Evaluate after 2 seconds since the model has to stabilize. Vehicle spawns a couple of meters above ground.
            # Stop evaluating when the time+time_step exceeds the time of the total simulation
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
                    while time2 < time + time_step:
                        temp_index = temp_index + 1
                        time2 = Slam.time[temp_index]

                    # use this data to linear interpolate the position
                    position1 = np.asarray(Slam.positions[time_index])
                    position2 = np.asarray(Slam.positions[temp_index])
                    # interpolated position
                    position_inter = time_step*(position2 - position1)/(time2-time)+position1

                    # use Slerp technique to linear interpolate the rotations
                    q1 = Quaternion(Slam.quaternions[time_index])
                    q2 = Quaternion(Slam.quaternions[temp_index])
                    time_inter = time_step/(time2-time)
                    q = Quaternion.slerp(q1, q2, time_inter)
                    # interpolated quaternion
                    q_inter = [q[0], q[1], q[2], q[3]]

                    # the interpolated homegeneous coordinate matrix
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
                RPE.append(RPE_i)

                # calculate the magnitude of translational error, normalized to the time step
                trans_err = math.sqrt(RPE_i[0][3]**2 + RPE_i[1][3]**2 + RPE_i[2][3]**2)/time_step
                trans_errs.append(trans_err)

                # calculate magnitude of angle
                a = RPE_i[0][0]  # roll
                b = RPE_i[1][1]  # pitch
                c = RPE_i[2][2]  # yaw
                d = 0.5*(a+b+c-1)

                rot_err = math.acos(max(min(d, 1), -1))/time_step  # guarantees value between -1 and 1
                rot_errs.append(math.degrees(rot_err))

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

        # plt.figure("Q1Q2")
        # plt.subplot(3, 1, 1)
        # plt.plot(gt.timeQ1Q2, Q1Q2gtx, label=gt.label)
        # plt.plot(Slam.timeQ1Q2, Q1Q2x, label=Slam.label)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Change in pose x [m]")
        # # plt.plot(time_debug, Q1Q2_debugx, 'o', label='linear interpolated')
        #
        # plt.subplot(3, 1, 2)
        # plt.plot(gt.timeQ1Q2, Q1Q2gty, label=gt.label)
        # plt.plot(Slam.timeQ1Q2, Q1Q2y, label=Slam.label)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Change in pose y [m]")
        # # plt.plot(time_debug, Q1Q2_debugy, 'o', label='linear interpolated')
        #
        # plt.subplot(3, 1, 3)
        # plt.plot(gt.timeQ1Q2, Q1Q2gtz, label=gt.label)
        # plt.plot(Slam.timeQ1Q2, Q1Q2z, label=Slam.label)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Change in pose z [m]")
        # # plt.plot(time_debug, Q1Q2_debugz, 'o', label='linear interpolated')
        # plt.legend()

        plt.figure("RPE")
        plt.subplot(3, 1, 1)
        plt.plot(Slam.timeQ1Q2, RPEx, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("RPE x [m]")
        plt.subplot(3, 1, 2)
        plt.plot(Slam.timeQ1Q2, RPEy, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("RPE y [m]")
        plt.subplot(3, 1, 3)
        plt.plot(Slam.timeQ1Q2, RPEz, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("RPE z [m]")
        plt.legend()

        plt.figure("Error Magnitude")
        plt.subplot(2, 1, 1)
        plt.plot(Slam.timeQ1Q2, trans_errs, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("translational error [m/s]")

        plt.subplot(2, 1, 2)
        plt.plot(Slam.timeQ1Q2, rot_errs, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("rotational error [deg/s]")
        plt.legend()




def main():

    method_gt = "gt"

    gt_file_static = "/home/sietse/SL_58_NV_0_SV_1_gt.txt"
    gt_file_dynamic = "/home/sietse/SL_58_NV_40_SV_1_gt.txt"

    with CarlaSlamEvaluate(method_gt, gt_file_static) as gt_static:
        gt_static.process_data()

    with CarlaSlamEvaluate(method_gt, gt_file_dynamic) as gt_dynamic:
        gt_dynamic.process_data()

    method_orb = "orb"
    orb_file_static = "/home/sietse/SL_58_NV_0_SV_1_orb.txt"
    orb_file_dynamic = "/home/sietse/SL_58_NV_40_SV_1_orb.txt"

    with CarlaSlamEvaluate(method_orb, orb_file_static) as orb_static:
        orb_static.process_data()

    with CarlaSlamEvaluate(method_orb, orb_file_dynamic) as orb_dynamic:
        orb_dynamic.process_data()

    time_step = 1

    gt_consistency(gt_dynamic, gt_static)
    evaluate_objects = [gt_static, orb_static, orb_dynamic]
    compare_position(evaluate_objects)
    # compare_quaternions(evaluate_objects)
    compare_euler_angles(evaluate_objects)
    evaluate_trajectory(evaluate_objects)
    # evaluate_pose_over_time([gt_static], [orb_static, orb_dynamic])
    evaluate_RPE([gt_static, gt_dynamic], [orb_static, orb_dynamic], time_step=time_step)
    plt.show()


if __name__=="__main__":
    main()
