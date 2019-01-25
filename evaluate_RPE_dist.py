from CarlaSlamPerformance import CarlaSlamEvaluate
import math
import numpy as np
from matplotlib import pyplot as plt


def evaluate_RPE_dist(GT, SLAM, eva_dist=float):
    """Input a list of CarlaSlamEvaluate ground truths and a list of CarlaSlamEvaluate SLAM pose estimations,
    it evaluates the Relative Pose Error over a given distance"""

    # Check if every method has a ground truth
    if len(GT) != len(SLAM):
        print("Not every SLAM method has a ground truth")
        return

    # get the ground truth and Slam data from the list
    for gt, Slam in zip(GT, SLAM):
        # compute the traveled distance based on the ground truth
        # list of traveled distances over time up until that point
        # note that distances is indexed according to the ground truth file
        distances = []
        for index, position in enumerate(gt.positions):
            if index == 0:
                distance = 0
                distances.append(distance)
            else:
                dx = position[0] - gt.positions[index-1][0]
                dy = position[1] - gt.positions[index-1][1]
                distance = distance + math.sqrt(dx**2 + dy**2)
                distances.append(distance)

        # contains the traveled distance for each Slam timestamp
        slam_distances = []
        for time in Slam.time:
            # find the equivalent time index in ground truth from SLAM time
            try:
                gt_index = gt.time.index(time)
            except ValueError:
                continue
            slam_distances.append(distances[gt_index])

        time_used = []
        slamQ1Q2 = []
        Q1Q2_gt = []
        RPE = []
        RPE_x = []
        RPE_y = []
        RPE_z = []
        trans_errs = []
        rot_errs = []
        for index, distance in enumerate(slam_distances):
            # find the index where the distance is larger than the current distance + evaluation distance
            # this will be the evaluation index
            index_eva = index

            try:
                while slam_distances[index_eva] < (distance + eva_dist):
                    index_eva = index_eva + 1
                time_used.append(Slam.time[index])
                time_eva = Slam.time[index_eva]
                Q2 = Slam.Q[index_eva]
                Q1 = Slam.Q[index]
                Q1_inv = np.linalg.inv(Q1)
                slamQ1Q2_i = Q1_inv.dot(Q2)
                slamQ1Q2.append(slamQ1Q2_i)

                # gt equivalent Q2 index
                gt_index_Q2 = gt.time.index(time_eva)
                # gt equivalent Q1 index
                gt_index_Q1 = gt.time.index(Slam.time[index])

                Q1_gt = gt.Q[gt_index_Q1]
                Q1_gt_inv = np.linalg.inv(Q1_gt)
                Q2_gt = gt.Q[gt_index_Q2]
                Q1Q2_gt_i = Q1_gt_inv.dot(Q2_gt)
                Q1Q2_gt.append(Q1Q2_gt_i)
                Q1Q2_gt_i_inv = np.linalg.inv(Q1Q2_gt_i)
                RPE_i = Q1Q2_gt_i_inv.dot(slamQ1Q2_i)

                # Calculate magnitude translational error
                RPE_x_i = RPE_i[0][3]
                RPE_x.append(RPE_x_i)
                RPE_y_i = RPE_i[1][3]
                RPE_y.append(RPE_y_i)
                RPE_z_i = RPE_i[2][3]
                RPE_z.append(RPE_z_i)
                trans_err = math.sqrt(RPE_x_i ** 2 + RPE_y_i ** 2 + RPE_z_i ** 2) / eva_dist
                trans_errs.append(trans_err)

                # Calculate magnitude of angle
                a = RPE_i[0][0]  # roll
                b = RPE_i[1][1]  # pitch
                c = RPE_i[2][2]  # yaw
                d = 0.5 * (a + b + c - 1)

                rot_err = math.acos(max(min(d, 1), -1)) / eva_dist

                rot_errs.append(rot_err)
                RPE.append(RPE_i)

            except IndexError:
                continue

        plt.figure("RPE translational error dissected")
        plt.subplot(3, 1, 1)
        plt.plot(time_used, RPE_x, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("translational error x [%] ")

        plt.subplot(3, 1, 2)
        plt.plot(time_used, RPE_y, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("translational error y [%] ")

        plt.subplot(3, 1, 3)
        plt.plot(time_used, RPE_z, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("translational error z [%] ")
        plt.legend()

        plt.figure("RPE Magnitude over distance")
        plt.subplot(2, 1, 1)
        plt.plot(time_used, trans_errs, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("translational error [%]")

        plt.subplot(2, 1, 2)
        plt.plot(time_used, rot_errs, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel("rotational error [deg/m]")
        plt.legend()


def main():
    method_gt = "gt"
    gt_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_gt.txt"
    gt_ps_static = 'k-'
    gt_file_dynamic = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_40_SV_1_gt.txt"
    gt_ps_dynamic = 'k-.'

    with CarlaSlamEvaluate(method_gt, gt_file_static, gt_ps_static) as gt_static:
        gt_static.process_data()

    with CarlaSlamEvaluate(method_gt, gt_file_dynamic, gt_ps_dynamic) as gt_dynamic:
        gt_dynamic.process_data()

    method_orb = "orb"
    orb_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_orb.txt"
    orb_ps_static = 'b-'
    orb_file_dynamic = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_40_SV_1_orb.txt"
    orb_ps_dynamic = 'r-'
    with CarlaSlamEvaluate(method_orb, orb_file_static, orb_ps_static) as orb_static:
        orb_static.process_data()

    with CarlaSlamEvaluate(method_orb, orb_file_dynamic, orb_ps_dynamic) as orb_dynamic:
        orb_dynamic.process_data()

    distance = 50

    plt.rcParams['axes.grid'] = True
    # evaluate_RPE_dist([gt_dynamic], [orb_dynamic], distance)
    evaluate_RPE_dist([gt_static, gt_dynamic], [orb_static, orb_dynamic], distance)
    plt.show()


if __name__=="__main__":
    main()
