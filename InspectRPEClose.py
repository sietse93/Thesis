import json
from func_Convert2Json import json2crf
from evaluate_pose import *
from class_ScenarioLocationPerformance import *
from matplotlib import pyplot as plt
import pdb
from main_InspectData import InspectJsonFileInDir
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
import numpy as np

def main_oppositeroad():
    Town = 1
    SL = 0
    nr_vans = 10
    base_dir = "/home/sietse/results_carla0.9/VansOppositeRoad/"
    base_dir_stat = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
    dir_name = "T{}_SL{}_d{}/".format(Town, SL, nr_vans)

    ev_dis = 10
    i = 0

    orb_static_list, gt = InspectJsonFileInDir(Town, SL, base_dir_stat, dir_name_stat, "SLAM")
    orb_dynamic_list, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name, "MC")

    for orb_stat, orb_dyn in zip(orb_static_list, orb_dynamic_list):

        stat_time, stat_trans_err, stat_rot_err = evaluate_RPE_dist(gt, orb_stat, ev_dis)
        dyn_time, dyn_trans_err, dyn_rot_err = evaluate_RPE_dist(gt, orb_dyn, ev_dis)

        plt.figure("RPE magnitude over {} m".format(ev_dis))
        plt.subplot(2, 1, 1)
        if i == 0:
            plt.plot(stat_time, stat_trans_err, 'b-', label="static")
            plt.plot(dyn_time, dyn_trans_err, 'r--', label="dynamic")
        else:
            plt.plot(stat_time, stat_trans_err, 'b-')
            plt.plot(dyn_time, dyn_trans_err, 'r--')
        plt.xlabel("time [s]", fontsize=14)
        plt.ylabel("translational error [-]", fontsize=14)
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(stat_time, stat_rot_err, 'b-')
        plt.plot(dyn_time, dyn_rot_err, 'r--')
        plt.xlabel("time [s]", fontsize=14)
        plt.ylabel("rotational error [deg/m]", fontsize=14)

        i += 1
    plt.legend()
    plt.show()



def main_stuckbehindvan():
    """Trying to proof that the corners in the trajectory cause the error to increase """
    Town = 1
    SL = 0
    dist = 10

    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
    dir_name_dyn = "T{}_SL{}_d{}/".format(Town, SL, dist)

    orb_static_list, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "VO")
    orb_dynamic_list, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "VO")

    ev_dis = 10
    i = 0
    for orb_stat, orb_dyn in zip(orb_static_list, orb_dynamic_list):

        stat_time, stat_trans_err, stat_rot_err = evaluate_RPE_dist(gt, orb_stat, ev_dis)
        dyn_time, dyn_trans_err, dyn_rot_err = evaluate_RPE_dist(gt, orb_dyn, ev_dis)

        plt.figure("RPE magnitude over {} m".format(ev_dis))
        plt.subplot(2, 1, 1)
        if i== 0:
            plt.plot(stat_time, stat_trans_err, 'b-', label="static")
            plt.plot(dyn_time, dyn_trans_err, 'r--', label="dynamic")
        else:
            plt.plot(stat_time, stat_trans_err, 'b-')
            plt.plot(dyn_time, dyn_trans_err, 'r--')
        plt.xlabel("time [s]", fontsize=14)
        plt.ylabel("translational error [-]", fontsize=14)
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(stat_time, stat_rot_err, 'b-')
        plt.plot(dyn_time, dyn_rot_err, 'r--')
        plt.xlabel("time [s]", fontsize=14)
        plt.ylabel("rotational error [deg/m]", fontsize=14)

        i += 1
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main_stuckbehindvan()