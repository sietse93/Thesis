from CarlaSlamPerformance import CarlaSlamEvaluate
import numpy as np
from matplotlib import pyplot as plt
import tf
import math
import evaluate_pose


class AverageSlamEvaluate:
    def __init__(self, label, plotstyle):
        self.label = label
        self.plotstyle = plotstyle
        self.time = []
        self.positions = []
        self.orientations = []
        self.quaternions = []
        self.Q = []


def average_orb(SLAM, timestep, label):
    """Input a list of CarlaSlamEvaluate objects from the orb data. The function averages the value of the pose
    for the timestamps that are available."""
    last_timestamp = SLAM[0].time[-1]
    cur_time = 0
    AverageSlam = AverageSlamEvaluate(label, 'r-')

    # loop through all timestamps
    while cur_time != last_timestamp:
        position = np.array([0, 0, 0])
        orientation = np.array([0, 0, 0])
        n_avg = 0.000
        for Slam in SLAM:
            try:
                eq_index = Slam.time.index(cur_time)
                position = position + Slam.positions[eq_index]
                orientation = orientation + Slam.orientations[eq_index]
                n_avg = n_avg+1
            except ValueError:
                # print(Slam.flocation, cur_time)
                continue
        # if there is data, take the average
        if n_avg > 4:
            # append average position
            AverageSlam.positions.append(position/n_avg)

            # append average orientation in degrees
            avg_orientation = orientation/n_avg
            avg_roll = avg_orientation[0]
            avg_pitch = avg_orientation[1]
            avg_yaw = avg_orientation[2]
            AverageSlam.orientations.append(avg_orientation)

            # append orientation in quaternions
            quaternion = tf.transformations.quaternion_from_euler(math.radians(avg_roll),
                                                                  math.radians(avg_pitch),
                                                                  math.radians(avg_yaw), axes='sxyz')
            AverageSlam.quaternions.append(quaternion)
            # append homogeneous coordinates
            q_avg = tf.transformations.euler_matrix(math.radians(avg_roll),
                                                math.radians(avg_pitch),
                                                math.radians(avg_yaw), axes='sxyz')
            q_avg[0][3] = position[0]
            q_avg[1][3] = position[1]
            q_avg[2][3] = position[2]
            AverageSlam.Q.append(q_avg)
            AverageSlam.time.append(cur_time)

        cur_time = round(cur_time + timestep, 3)

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
    AverageStatic = average_orb(SLAM, timestep, 'orb_static')
    SLAM.append(AverageStatic)
    # print(len(AverageStatic.time), len(AverageStatic.orientations))
    evaluate_pose.compare_position(SLAM)
    evaluate_pose.compare_euler_angles(SLAM)
    plt.show()


if __name__=="__main__":
    main()
