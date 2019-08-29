import json
from func_Convert2Json import json2crf
from evaluate_pose import *
from class_ScenarioLocationPerformance import *
from matplotlib import pyplot as plt
import pdb
from main_InspectData import InspectJsonFileInDir
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
import numpy as np


def main():

    Town = 1
    SL = 0
    nr_vans = 10
    dist = 10

    base_dir_opposite = "/home/sietse/results_carla0.9/VansOppositeRoad/"
    base_dir_stuck = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
    dir_name_opp = "T{}_SL{}_d{}/".format(Town, SL, nr_vans)
    dir_name_stuck = "T{}_SL{}_d{}/".format(Town, SL, dist)

    orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir_stuck, dir_name_stat, "SLAM")
    orb_opposite, gt_opp = InspectJsonFileInDir(Town, SL, base_dir_opposite, dir_name_opp, "SLAM")
    orb_stuck, gt_stuck = InspectJsonFileInDir(Town, SL, base_dir_stuck, dir_name_stuck, "VO")

    # methods = [gt]
    # methods.extend(orb_static)
    # methods.extend(orb_opposite)
    # methods.extend(orb_stuck)
    # evaluate_trajectory(methods)

    static_mean, static_std = RmseRpe(orb_static, gt)
    opposite_mean, opposite_std = RmseRpe(orb_opposite, gt)
    stuck_mean, stuck_std = RmseRpe(orb_stuck, gt)

    pdb.set_trace()


def RmseRpe(ORB, gt):
    """Calculates the mean value of RMSE values of an ORB list"""

    rmse_trans_list = []
    rmse_rot_list = []

    for orb in ORB:
        t, trans, rot = evaluate_RPE_dist(gt, orb, 100)
        rmse_trans = calc_rmse(trans)
        rmse_rot = calc_rmse(rot)
        rmse_trans_list.append(rmse_trans)
        rmse_rot_list.append(rmse_rot)

    rmse_trans = np.array(rmse_trans_list)
    rmse_rot = np.array(rmse_rot_list)

    trans_mean = rmse_trans.mean()
    trans_std = rmse_trans.std()
    rot_mean = rmse_rot.mean()
    rot_std = rmse_rot.std()

    RMSE_mean = (trans_mean, rot_mean)
    RMSE_std = (trans_std, rot_std)

    return RMSE_mean, RMSE_std


if __name__ == "__main__" :
    main()
