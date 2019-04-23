import math
import numpy as np


def evaluate_RPE_dist(gt, Slam, eva_dist=100.0):
    # compute the traveled distance based on the ground truth
    # list of traveled distances over time up until that point
    # note that distances is indexed according to the ground truth file
    distances = []
    for index, position in enumerate(gt.positions):
        if index == 0:
            distance = 0
            distances.append(distance)
        else:
            dx = position[0] - gt.positions[index - 1][0]
            dy = position[1] - gt.positions[index - 1][1]
            distance = distance + math.sqrt(dx ** 2 + dy ** 2)
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

            # Calculate magnitude translational error and normalize to distance
            RPE_x_i = RPE_i[0][3]
            RPE_x.append(RPE_x_i)
            RPE_y_i = RPE_i[1][3]
            RPE_y.append(RPE_y_i)
            RPE_z_i = RPE_i[2][3]
            RPE_z.append(RPE_z_i)
            trans_err = math.sqrt(RPE_x_i ** 2 + RPE_y_i ** 2 + RPE_z_i ** 2) / eva_dist
            trans_errs.append(trans_err)  # ??trans_err is now a ratio, multiply by 100 for percentage

            # Calculate magnitude of angle and normalize to distance
            a = RPE_i[0][0]  # roll
            b = RPE_i[1][1]  # pitch
            c = RPE_i[2][2]  # yaw
            d = 0.5 * (a + b + c - 1)

            rot_err = math.degrees(math.acos(max(min(d, 1), -1))) / eva_dist

            rot_errs.append(rot_err)
            RPE.append(RPE_i)

        except IndexError:
            continue

        return time_used, trans_errs, rot_errs


def calc_rmse(error_list):
    return np.sqrt(np.sum(np.array(error_list)**2)/len(error_list))
