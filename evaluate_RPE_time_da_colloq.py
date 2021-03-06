from CarlaSlamPerformance import CarlaSlamEvaluate
import numpy as np
from matplotlib import pyplot as plt
import tf
from pyquaternion import Quaternion
import math
from plot_encountered_vehicles import *


def evaluate_RPE_time_da_colloq(GT, SLAM, time_step=float, *args):
    """Input a list of CarlaSlamEvaluate ground truths and a list of CarlaSlamEvaluate SLAM pose estimations.
    Optional, plot the encountered vehicles in the plots"""

    # Check if every method has a ground truth
    if len(GT) != len(SLAM):
        print("Not every SLAM method has a ground truth")
        return

    # Evaluate Relative Pose Error
    # loop through each Slam method with associated ground truth
    for gt, Slam in zip(GT, SLAM):
        RPE = []
        time_debug = []
        time_inter_debug = []
        Q1Q2_debug = []

        trans_errs = []
        rot_errs = []

        pos_debug_inter = []
        quaternion_debug_inter = []

        end_time = Slam.time.index(106)

        for time in Slam.time[:end_time]:
            # Evaluate after 2 seconds since the model has to stabilize. Vehicle spawns a couple of meters above ground.
            # Stop evaluating when the time+time_step exceeds the time of the total simulation
            if time > 2 and time < (Slam.time[-1]-time_step):
                time_index = Slam.time.index(time)
                Q1 = Slam.Q[time_index]
                Q1_inv = np.linalg.inv(Q1)

                try:
                    Q2 = Slam.Q[Slam.time.index(round(time+time_step, 3))]  # add time_step if it doesnt exist it gets a ValueError
                except ValueError:
                    continue
                    # # if time step does not work, find the closest timestamp that is bigger than the time step
                    # temp_index = time_index+1
                    # time2 = Slam.time[temp_index]
                    # while time2 < time + time_step:
                    #     temp_index = temp_index + 1
                    #     time2 = Slam.time[temp_index]
                    #
                    # # use this data to linear interpolate the position
                    # position1 = np.asarray(Slam.positions[time_index])
                    # position2 = np.asarray(Slam.positions[temp_index])
                    # # interpolated position
                    # position_inter = time_step*(position2 - position1)/(time2-time)+position1
                    # pos_debug_inter.append(position_inter)
                    #
                    # # use Slerp technique to linear interpolate the rotations
                    # q1 = Quaternion(Slam.quaternions[time_index])
                    # q2 = Quaternion(Slam.quaternions[temp_index])
                    # time_inter = time_step/(time2-time)
                    # q = Quaternion.slerp(q1, q2, time_inter)
                    # # interpolated quaternion
                    # q_inter = [q[0], q[1], q[2], q[3]]
                    # quaternion_debug_inter.append(q_inter)
                    #
                    # # the interpolated homegeneous coordinate matrix
                    # Q2 = tf.transformations.quaternion_matrix(q_inter)
                    # Q2[0][3] = position_inter[0]
                    # Q2[1][3] = position_inter[1]
                    # Q2[2][3] = position_inter[2]
                    #
                    # # some debug variables that allows you to visualize the linear interpolated data
                    # # time to check the interpolated data for Q1Q2
                    # time_debug.append(time)
                    # # time to check the interpolated pose (position and quaternions)
                    # time_inter_debug.append(time+1)
                    # Q1Q2_debug.append(Q1_inv.dot(Q2))

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

                rot_err = math.degrees(math.acos(max(min(d, 1), -1)))/time_step  # guarantees value between -1 and 1
                rot_errs.append(math.degrees(rot_err))

        # # check if the interpolated position makes sense
        # inter_x = [position[0] for position in pos_debug_inter]
        # inter_y = [position[1] for position in pos_debug_inter]
        # inter_z = [position[2] for position in pos_debug_inter]
        #
        # x = [position[0] for position in Slam.positions]
        # y = [position[1] for position in Slam.positions]
        # z = [position[2] for position in Slam.positions]
        #
        # plt.figure("interpolated position")
        # plt.subplot(3, 1, 1)
        # plt.plot(time_inter_debug, inter_x, 'x')
        # plt.plot(Slam.time, x)
        #
        # plt.subplot(3, 1, 2)
        # plt.plot(time_inter_debug, inter_y, 'x')
        # plt.plot(Slam.time, y)
        #
        # plt.subplot(3, 1, 3)
        # plt.plot(time_inter_debug, inter_z, 'x')
        # plt.plot(Slam.time, z)
        #
        # inter_q1 = [quaternion[0] for quaternion in quaternion_debug_inter]
        # inter_q2 = [quaternion[1] for quaternion in quaternion_debug_inter]
        # inter_q3 = [quaternion[2] for quaternion in quaternion_debug_inter]
        # inter_q4 = [quaternion[3] for quaternion in quaternion_debug_inter]
        #
        # q1 = [quaternion[0] for quaternion in Slam.quaternions]
        # q2 = [quaternion[1] for quaternion in Slam.quaternions]
        # q3 = [quaternion[2] for quaternion in Slam.quaternions]
        # q4 = [quaternion[3] for quaternion in Slam.quaternions]
        #
        # plt.figure("interpolated quaternions")
        # plt.subplot(4, 1, 1)
        # plt.plot(time_inter_debug, inter_q1, 'x', label="interpolated")
        # plt.plot(Slam.time, q1, label=Slam.label)
        #
        # plt.subplot(4, 1, 2)
        # plt.plot(time_inter_debug, inter_q2, 'x', label="interpolated")
        # plt.plot(Slam.time, q2, label=Slam.label)
        #
        # plt.subplot(4, 1, 3)
        # plt.plot(time_inter_debug, inter_q3, 'x', label="interpolated")
        # plt.plot(Slam.time, q3, label=Slam.label)
        #
        # plt.subplot(4, 1, 4)
        # plt.plot(time_inter_debug, inter_q4, 'x', label="interpolated")
        # plt.plot(Slam.time, q4, label=Slam.label)
        # plt.legend()
        #
        RPEx = [matrix[0][3] for matrix in RPE]
        RPEy = [matrix[1][3] for matrix in RPE]
        RPEz = [matrix[2][3] for matrix in RPE]
        #
        # Q1Q2x = [matrix[0][3] for matrix in Slam.Q1Q2]
        # Q1Q2y = [matrix[1][3] for matrix in Slam.Q1Q2]
        # Q1Q2z = [matrix[2][3] for matrix in Slam.Q1Q2]
        #
        # Q1Q2gtx = [matrix[0][3] for matrix in gt.Q1Q2]
        # Q1Q2gty = [matrix[1][3] for matrix in gt.Q1Q2]
        # Q1Q2gtz = [matrix[2][3] for matrix in gt.Q1Q2]
        #
        # Q1Q2_debugx = [matrix[0][3] for matrix in Q1Q2_debug]
        # Q1Q2_debugy = [matrix[1][3] for matrix in Q1Q2_debug]
        # Q1Q2_debugz = [matrix[2][3] for matrix in Q1Q2_debug]
        #
        # plt.figure("Q1Q2")
        # plt.subplot(3, 1, 1)
        # plt.plot(gt.timeQ1Q2, Q1Q2gtx, label=gt.label)
        # plt.plot(Slam.timeQ1Q2, Q1Q2x, label=Slam.label)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Change in pose x [m]")
        # plt.plot(time_debug, Q1Q2_debugx, 'o', label='linear interpolated')
        #
        # plt.subplot(3, 1, 2)
        # plt.plot(gt.timeQ1Q2, Q1Q2gty, label=gt.label)
        # plt.plot(Slam.timeQ1Q2, Q1Q2y, label=Slam.label)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Change in pose y [m]")
        # plt.plot(time_debug, Q1Q2_debugy, 'o', label='linear interpolated')
        #
        # plt.subplot(3, 1, 3)
        # plt.plot(gt.timeQ1Q2, Q1Q2gtz, label=gt.label)
        # plt.plot(Slam.timeQ1Q2, Q1Q2z, label=Slam.label)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Change in pose z [m]")
        # plt.plot(time_debug, Q1Q2_debugz, 'o', label='linear interpolated')
        # plt.legend()

        # plt.figure("RPE over time")
        # plt.subplot(3, 1, 1)
        # plt.plot(Slam.timeQ1Q2, RPEx, Slam.plotstyle, label=Slam.label)
        # plot_vehicles_encountered(args[0])
        # plt.xlabel("time [s]")
        # plt.ylabel("RPE x [m]")
        # plt.subplot(3, 1, 2)
        # plt.plot(Slam.timeQ1Q2, RPEy, Slam.plotstyle, label=Slam.label)
        # plot_vehicles_encountered(args[0])
        # plt.xlabel("time [s]")
        # plt.ylabel("RPE y [m]")
        # plt.subplot(3, 1, 3)
        # plt.plot(Slam.timeQ1Q2, RPEz, Slam.plotstyle, label=Slam.label)
        # plot_vehicles_encountered(args[0])
        # plt.xlabel("time [s]")
        # plt.ylabel("RPE z [m]")
        # plt.legend()

        # trans_debug = [trans_errs[Slam.timeQ1Q2.index(time)] for time in time_debug]
        #rot_debug = [rot_errs[Slam.timeQ1Q2.index(time)] for time in time_debug]
        plt.figure("RPE Magnitude over time")
        plt.plot(Slam.timeQ1Q2, trans_errs, Slam.plotstyle, label=Slam.label)
        #plt.plot(time_debug, trans_debug, 'kx', label="interpolated data")
        plot_vehicles_encountered(args[0])
        plt.legend()
        plt.xlabel("time [s]")
        plt.ylabel("translational error [m/s]")



def main():
    method_gt = "gt"
    gt_ps_static = 'k-'
    gt_file_static = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_gt.txt"
    # gt_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_gt.txt"
    with CarlaSlamEvaluate(method_gt, gt_file_static, gt_ps_static) as gt_static:
        gt_static.process_data()

    method_orb = "orb"
    orb_ps_static = 'g-'
    orb_file_static = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_2.txt"
    # orb_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_orb.txt"

    with CarlaSlamEvaluate(method_orb, orb_file_static, orb_ps_static) as orb_static:
        orb_static.process_data()

    plt.rcParams['axes.grid'] = True

    GT = [gt_static]
    SLAM = [orb_static]
    time_step = 1.0

    evaluate_RPE_time(GT, SLAM, time_step)
    plt.show()

if __name__ == "__main__":
    main()
