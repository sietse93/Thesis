from CarlaSlamPerformance import CarlaSlamEvaluate
import numpy as np
from matplotlib import pyplot as plt
import tf
import math
import evaluate_pose
import AverageQuaternions
from pyquaternion import Quaternion


class AverageSlamEvaluate:
    def __init__(self, label, plotstyle):
        self.label = label
        self.plotstyle = plotstyle
        self.time = []
        self.positions = []
        self.orientations = []
        self.quaternions = []
        self.Q = []

        # The function only uses the data that has more than 4 data points on each timestamp. It could be that
        # large time gaps result. time_gap shows the biggest gap in time, between the timestamps.
        self.time_gap = ()

        # Relative pose change over a certain time frame expressed in homogeneous coordinates
        self.Q1Q2 = []

        # time stamps used for RPE
        self.timeQ1Q2 = []

        # Root Mean Square Error of Relative Pose Error over distance
        self.RPE_RMSE_dist = []


def average_orb(SLAM, timestep, label, plotstyle):
    """Input a list of CarlaSlamEvaluate objects from the orb data. The function averages the value of the pose
    for the timestamps that are available."""
    last_timestamp = SLAM[0].time[-1]
    cur_time = 0
    AverageSlam = AverageSlamEvaluate(label, plotstyle)

    # loop through all timestamps
    while cur_time != last_timestamp:
        position = np.array([0, 0, 0])
        orientation = np.array([0, 0, 0])
        n_avg = 0
        for Slam in SLAM:
            try:
                eq_index = Slam.time.index(cur_time)
                position = position + Slam.positions[eq_index]
                orientation = orientation + Slam.orientations[eq_index]
                if n_avg == 0:
                    quat = np.array(Slam.quaternions[eq_index])
                else:
                    quat = np.vstack([quat, Slam.quaternions[eq_index]])
                n_avg = n_avg+1
            except ValueError:
                # print(Slam.flocation, cur_time)
                continue
        # if there is data, take the average
        if n_avg == len(SLAM):
            average_position = position/n_avg
            AverageSlam.positions.append(average_position)

            # We need to use Slerp to average quaternions. Which needs to be used to get the correct rotation matrix.
            # q0 = Quaternion(quat[0][:])
            # q1 = Quaternion(quat[1][:])
            # q2 = Quaternion(quat[2][:])
            # q3 = Quaternion(quat[3][:])
            # q4 = Quaternion(quat[4][:])
            # quat_avg = Quaternion.slerp(Quaternion.slerp(Quaternion.slerp(Quaternion.slerp(q0, q1), q2), q3), q4)
            # quat_avg_array = np.array([quat_avg[0], quat_avg[1], quat_avg[2], quat_avg[3]])
            # AverageSlam.quaternions.append(quat_avg_array)

            # q_avg = tf.transformations.quaternion_matrix(quat_avg_array)
            # q_avg[0][3] = average_position[0]
            # q_avg[1][3] = average_position[1]
            # q_avg[2][3] = average_position[2]
            # AverageSlam.Q.append(q_avg)

            # append average position, doesn't work. Lot of sign switching in results
            # # average quaternion
            # w = np.ones(n_avg)
            # quat_avg = AverageQuaternions.average_quat(quat, w)
            # AverageSlam.quaternions.append(quat_avg)

            # Euler angles, Note yaw probably only gives between 0-180  degrees
            # avg_roll, avg_pitch, avg_yaw = tf.transformations.euler_from_quaternion(quat_avg_array, axes='sxyz')
            # avg_orientation = np.array([avg_roll, avg_pitch, avg_yaw])
            # AverageSlam.orientations.append(avg_orientation)

            # append average orientation in degrees, if I convert Quaternion to euler angles it will give me wrong data.
            avg_orientation = orientation/n_avg
            AverageSlam.orientations.append(avg_orientation)
            avg_roll = avg_orientation[0]
            avg_pitch = avg_orientation[1]
            avg_yaw = avg_orientation[2]
            #
            # append orientation in quaternions
            quaternion = tf.transformations.quaternion_from_euler(math.radians(avg_roll),
                                                                  math.radians(avg_pitch),
                                                                  math.radians(avg_yaw), axes='sxyz')
            AverageSlam.quaternions.append(quaternion)
            # append homogeneous coordinates
            q_avg = tf.transformations.euler_matrix(math.radians(avg_roll), math.radians(avg_pitch), math.radians(avg_yaw), axes='sxyz')
            # q_avg = tf.transformations.quaternion_matrix(quaternion)
            q_avg[0][3] = average_position[0]
            q_avg[1][3] = average_position[1]
            q_avg[2][3] = average_position[2]
            AverageSlam.Q.append(q_avg)
            AverageSlam.time.append(cur_time)

        cur_time = round(cur_time + timestep, 3)

    AverageSlam.time_gap = 0
    for index, time_stamp in enumerate(AverageSlam.time[:-1]):
        if AverageSlam.time[index + 1] - time_stamp > AverageSlam.time_gap:
            AverageSlam.time_gap = AverageSlam.time[index + 1] - time_stamp
    return AverageSlam


def main():
    orb_static_1f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_1.txt"
    orb_static_2f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_2.txt"
    orb_static_3f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_3.txt"
    orb_static_4f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_4.txt"
    orb_static_5f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_5.txt"

    with CarlaSlamEvaluate("orb", orb_static_1f, 'r--') as orb_static_1:
        orb_static_1.process_data()

    with CarlaSlamEvaluate("orb", orb_static_2f, 'b--') as orb_static_2:
        orb_static_2.process_data()

    with CarlaSlamEvaluate("orb", orb_static_3f, 'c--') as orb_static_3:
        orb_static_3.process_data()

    with CarlaSlamEvaluate("orb", orb_static_4f, 'm--') as orb_static_4:
        orb_static_4.process_data()

    with CarlaSlamEvaluate("orb", orb_static_5f, 'g--') as orb_static_5:
        orb_static_5.process_data()

    timestep = 0.025
    SLAM = [orb_static_1, orb_static_2, orb_static_3, orb_static_4, orb_static_5]
    AverageStatic = average_orb(SLAM, timestep, 'orb_static', 'r-')
    SLAM.append(AverageStatic)
    # print(len(AverageStatic.time), len(AverageStatic.orientations))
    evaluate_pose.compare_position(SLAM)
    evaluate_pose.compare_euler_angles(SLAM)
    evaluate_pose.compare_quaternions(SLAM)

    plt.show()


if __name__=="__main__":
    main()
