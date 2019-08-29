import json
from func_Convert2Json import json2crf
from evaluate_pose import *
from class_ScenarioLocationPerformance import *
from matplotlib import pyplot as plt
import pdb
from main_InspectData import InspectJsonFileInDir
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
import numpy as np

Town = 1
SL = 27
dv = 10
ev_dis = 10
base_dir_stat = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
base_dir_dyn = "/home/sietse/results_carla0.9/VansOppositeRoad/"

dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
dir_name_dyn = "T{}_SL{}_d{}/".format(Town, SL, dv)

orb_static_list, gt = InspectJsonFileInDir(Town, SL, base_dir_stat, dir_name_stat, "SLAM")
orb_dynamic_list, gt = InspectJsonFileInDir(Town, SL, base_dir_dyn, dir_name_dyn, "SLAM")
orb_dynamic_list_mc, gt = InspectJsonFileInDir(Town, SL, base_dir_dyn, dir_name_dyn, "MC")


i = 0

for orb_stat, orb_dyn, orb_dyn_mc in zip(orb_static_list, orb_dynamic_list, orb_dynamic_list_mc):
    stat_time, stat_trans_err, stat_rot_err = evaluate_RPE_dist(gt, orb_stat, ev_dis)
    dyn_time, dyn_trans_err, dyn_rot_err = evaluate_RPE_dist(gt, orb_dyn, ev_dis)
    dyn_time_mc, dyn_trans_err_mc, dyn_rot_err_mc = evaluate_RPE_dist(gt, orb_dyn_mc, ev_dis)

    mc_plotstyle = orb_dyn_mc.plotstyle[:-2] + '.-'


    plt.figure("RPE magnitude over {} m".format(ev_dis))
    plt.subplot(2, 1, 1)
    # plt.plot(stat_time, stat_trans_err, orb_stat.plotstyle)
    # plt.plot(dyn_time, dyn_trans_err, orb_dyn.plotstyle)
    # plt.plot(dyn_time_mc, dyn_trans_err_mc, mc_plotstyle)
    # plt.plot(stat_time, stat_trans_err, color='black')
    plt.plot(dyn_time, dyn_trans_err, color='green')
    plt.plot(dyn_time_mc, dyn_trans_err_mc, color='red')

    plt.subplot(2, 1, 2)
    plt.plot(stat_time, stat_rot_err, orb_stat.plotstyle)
    plt.plot(dyn_time, dyn_rot_err, color='green')
    plt.plot(dyn_time_mc, dyn_rot_err_mc, color='red')

    i += 1

fs_x = 14
fs_y = 16
fs_title = 16
fs_legend = 12
plt.subplot(2,1,1)
plt.suptitle(r'Vans driving opposite road in Town 1 Nr 2: ORB SLAM Relative Pose Error over time ($\Delta$=10m)', fontsize=18, fontweight='bold', y=0.95)
plt.title("Translational component relative pose error", fontsize=fs_title, fontweight='bold')
plt.xticks(fontsize=fs_x)
plt.yticks(fontsize=fs_x)
plt.xlabel("Time [s]", fontsize=fs_y)
plt.ylabel("Translational error [m/m]", fontsize=fs_y)
plt.plot([], [], 'g-', label="Vans driving on opposite road: ORB SLAM")
plt.plot([], [], 'r-', label="Vans driving on opposite road: ORB MC")
# plt.scatter([], [], marker='x', s=100, color='black', linewidths=5, label="Visual example see image (b)")
# plt.scatter([], [], marker='+', s=100, color='black', label="Visual example see image (c)")
plt.legend(fontsize=fs_legend)

plt.subplot(2, 1, 2)
plt.title("Rotational component relative pose error", fontsize=fs_title, fontweight='bold')
plt.xlabel("Time [s]", fontsize=fs_y)
plt.ylabel("Rotational error [deg/m]", fontsize=fs_y)
plt.xticks(fontsize=fs_x)
plt.yticks(fontsize=fs_x)

plt.show()