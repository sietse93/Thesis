import json
from func_Convert2Json import json2crf
from evaluate_pose import *
from class_ScenarioLocationPerformance import *
from matplotlib import pyplot as plt
import pdb
from main_InspectData import InspectJsonFileInDir
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
import numpy as np

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
        plt.plot(stat_time, stat_trans_err, orb_stat.plotstyle)
        plt.plot(dyn_time, dyn_trans_err, orb_dyn.plotstyle)

        plt.subplot(2, 1, 2)
        plt.plot(stat_time, stat_rot_err, orb_stat.plotstyle)
        plt.plot(dyn_time, dyn_rot_err, orb_dyn.plotstyle)

        # add markers at the position where visual examples are inserted.
        if i == 1:
            for index, time in enumerate(dyn_time):
                if time == 25.55:
                    imageB_t = time
                    imageB_trans = dyn_trans_err[index]
                    imageB_rot = dyn_rot_err[index]
                elif time == 82.0:
                    imageC_t = time
                    imageC_trans = dyn_trans_err[index]
                    imageC_rot = dyn_rot_err[index]
            ms = 15
            mew = 3
            plt.subplot(2, 1, 1)
            plt.plot([imageB_t], [imageB_trans], marker='x', markersize=ms, markeredgewidth=mew, color='black')
            plt.plot([imageC_t], [imageC_trans], marker='+', markersize=ms, markeredgewidth=mew, color='black')
            plt.subplot(2, 1, 2)
            plt.plot([imageB_t], [imageB_rot], marker='x', markersize=ms, markeredgewidth=mew, color='black')
            plt.plot([imageC_t], [imageC_rot], marker='+', markersize=ms, markeredgewidth=mew, color='black')

        i += 1




    fs_x = 14
    fs_y = 16
    fs_title = 16
    fs_legend = 12
    plt.subplot(2,1,1)
    plt.suptitle(r'Town 1 Nr 1: ORB VO Relative Pose Error over time ($\Delta$=10m)', fontsize=18, fontweight='bold', y=0.95)
    plt.title("Translational component relative pose error", fontsize=fs_title, fontweight='bold')
    plt.xticks(fontsize=fs_x)
    plt.yticks(fontsize=fs_x)
    plt.xlabel("Time [s]", fontsize=fs_y)
    plt.ylabel("Translational error [m/m]", fontsize=fs_y)
    plt.plot([], [], 'k-', label="Static")
    plt.plot([], [], 'k--', label="Driving behind van \ninter-vehicle distance = 10 meters")
    plt.scatter([], [], marker='x', s=100, color='black', linewidths=5, label="Visual example see image (b)")
    plt.scatter([], [], marker='+', s=100, color='black', label="Visual example see image (c)")
    plt.legend(fontsize=fs_legend)

    plt.subplot(2, 1, 2)
    plt.title("Rotational component relative pose error", fontsize=fs_title, fontweight='bold')
    plt.xlabel("Time [s]", fontsize=fs_y)
    plt.ylabel("Rotational error [deg/m]", fontsize=fs_y)
    plt.xticks(fontsize=fs_x)
    plt.yticks(fontsize=fs_x)

    plt.show()

if __name__ == "__main__":
    main_stuckbehindvan()