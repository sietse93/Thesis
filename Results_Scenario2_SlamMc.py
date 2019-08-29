from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb

base_dir_dyn = "/home/sietse/results_carla0.9/VansOppositeRoad/"
base_dir_stat = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
# dynamic variable (10 cars on opposite lane)
dv = 10

mode = "SLAM"
mode_mc = "MC"

x = []
x_scatter = []

avg_trans = []
avg_trans_upp = []
avg_trans_low = []
avg_rot = []
avg_rot_upp = []
avg_rot_low = []
track_fail = []
loop_fail = []
scatter_trans = []
scatter_rot = []

avg_trans_mc = []
avg_trans_upp_mc = []
avg_trans_low_mc = []
avg_rot_mc = []
avg_rot_upp_mc = []
avg_rot_low_mc = []
track_fail_mc = []
loop_fail_mc = []
scatter_trans_mc = []
scatter_rot_mc = []

total_SL = 0
Towns = (1,2, 3)

for Town in Towns:

    if Town == 1:
        starting_locations = (0, 27, 58)
    elif Town == 2:
        starting_locations = (18, 37, 78)
    elif Town == 3:
        starting_locations = (75, 97, 127, 132)
    else:
        print("Town does not exist")

    total_SL += len(starting_locations)

    for index, SL in enumerate(starting_locations):
        dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
        dir_name_dyn = "T{}_SL{}_d{}/".format(Town, SL, dv)

        # get the static data from the stuck behind van experiment
        orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir_stat, dir_name_stat, "SLAM")
        orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir_dyn, dir_name_dyn, "SLAM")
        Perf = ScenarioLocationPerformance(dv, Town, SL, orb_static, orb_dynamic, gt)

        orb_static_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir_stat, dir_name_stat, "MC")
        orb_dynamic_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir_dyn, dir_name_dyn, "MC")
        Perf_mc = ScenarioLocationPerformance(dv, Town, SL, orb_static_mc, orb_dynamic_mc, gt_mc)

        x.extend(["T" + str(Perf.location["Town"]) + "Nr" + str(index + 1) + "\n" + " Static",
              "T" + str(Perf.location["Town"]) + "Nr" + str(index + 1) + "\n" + "Dynamic"])
        avg_trans.extend([Perf.rmse_static_avg[0], Perf.rmse_dynamic_avg[0]])
        avg_trans_upp.extend([Perf.rmse_static_avg[0] + 2 * Perf.rmse_static_std[0],
                              Perf.rmse_dynamic_avg[0] + 2 * Perf.rmse_dynamic_std[0]])
        avg_trans_low.extend([Perf.rmse_static_avg[0] - 2 * Perf.rmse_static_std[0],
                              Perf.rmse_dynamic_avg[0] - 2 * Perf.rmse_dynamic_std[0]])

        raw_trans_static = [raw_rmse_static[0] for raw_rmse_static in Perf.rmse_static]
        raw_trans_dynamic = [raw_rmse_dynamic[0] for raw_rmse_dynamic in Perf.rmse_dynamic]
        scatter_trans.extend([raw_trans_static, raw_trans_dynamic])

        avg_rot.extend([Perf.rmse_static_avg[1], Perf.rmse_dynamic_avg[1]])
        avg_rot_upp.extend([Perf.rmse_static_avg[1] + 2 * Perf.rmse_static_std[1],
                            Perf.rmse_dynamic_avg[1] + 2 * Perf.rmse_dynamic_std[1]])
        avg_rot_low.extend([Perf.rmse_static_avg[1] - 2 * Perf.rmse_static_std[1],
                            Perf.rmse_dynamic_avg[1] - 2 * Perf.rmse_dynamic_std[1]])

        raw_rot_static = [raw_rmse_static[1] for raw_rmse_static in Perf.rmse_static]
        raw_rot_dynamic = [raw_rmse_dynamic[1] for raw_rmse_dynamic in Perf.rmse_dynamic]
        scatter_rot.extend([raw_rot_static, raw_rot_dynamic])

        track_fail.extend([Perf.lost_track_static, Perf.lost_track_dynamic])
        loop_fail.extend([Perf.false_loop_static, Perf.false_loop_dynamic])

        # MC data
        avg_trans_mc.extend([Perf_mc.rmse_static_avg[0], Perf_mc.rmse_dynamic_avg[0]])
        avg_trans_upp_mc.extend([Perf_mc.rmse_static_avg[0] + 2 * Perf_mc.rmse_static_std[0],
                                 Perf_mc.rmse_dynamic_avg[0] + 2 * Perf_mc.rmse_dynamic_std[0]])
        avg_trans_low_mc.extend([Perf_mc.rmse_static_avg[0] - 2 * Perf_mc.rmse_static_std[0],
                                 Perf_mc.rmse_dynamic_avg[0] - 2 * Perf_mc.rmse_dynamic_std[0]])

        raw_trans_static_mc = [raw_rmse_static_mc[0] for raw_rmse_static_mc in Perf_mc.rmse_static]
        raw_trans_dynamic_mc = [raw_rmse_dynamic_mc[0] for raw_rmse_dynamic_mc in Perf_mc.rmse_dynamic]
        scatter_trans_mc.extend([raw_trans_static_mc, raw_trans_dynamic_mc])

        avg_rot_mc.extend([Perf_mc.rmse_static_avg[1], Perf_mc.rmse_dynamic_avg[1]])
        avg_rot_upp_mc.extend([Perf_mc.rmse_static_avg[1] + 2 * Perf_mc.rmse_static_std[1],
                               Perf_mc.rmse_dynamic_avg[1] + 2 * Perf_mc.rmse_dynamic_std[1]])
        avg_rot_low_mc.extend([Perf_mc.rmse_static_avg[1] - 2 * Perf_mc.rmse_static_std[1],
                               Perf_mc.rmse_dynamic_avg[1] - 2 * Perf_mc.rmse_dynamic_std[1]])

        raw_rot_static_mc = [raw_rmse_static_mc[1] for raw_rmse_static_mc in Perf_mc.rmse_static]
        raw_rot_dynamic_mc = [raw_rmse_dynamic_mc[1] for raw_rmse_dynamic_mc in Perf_mc.rmse_dynamic]
        scatter_rot_mc.extend([raw_rot_static_mc, raw_rot_dynamic_mc])

        track_fail_mc.extend([Perf_mc.lost_track_static, Perf_mc.lost_track_dynamic])
        loop_fail_mc.extend([Perf_mc.false_loop_static, Perf_mc.false_loop_dynamic])

track_fail = [cell*100.0 for cell in track_fail]
loop_fail = [cell*100.0 for cell in loop_fail]
track_fail_mc = [cell*100.0 for cell in track_fail_mc]
loop_fail_mc = [cell*100.0 for cell in loop_fail_mc]

# sets the size axis labels
fs_x = 14
# sets the size of the texts
fs_y = 16
# set size of legend
f_legend = 12
fs_title = 16
x_lim_right = len(x)-0.5
x_lim_left = -0.5

total_scenarios = 2

plt.figure("Performance SLAM Algorithm")
plt.suptitle("Vehicles driving on opposite road: ORB SLAM and without map point culling (ORB MC)", fontsize=18, fontweight='bold', y=0.95)
# plt.suptitle("Results Town {}: ORB SLAM and without map point culling (ORB MC)".format(Town), fontsize=18, fontweight='bold', y=0.95)
ind = np.arange(len(x))

plt.subplot(3, 1, 1)
for j in range(total_SL):
    plt.plot(ind[j*total_scenarios:j*total_scenarios+total_scenarios],
             np.array(avg_trans[j*total_scenarios:j*total_scenarios+total_scenarios]),
             color='green')
    plt.fill_between(ind[j*total_scenarios:j*total_scenarios+total_scenarios],
                     np.array(avg_trans_low[j*total_scenarios:j*total_scenarios+total_scenarios]),
                     np.array(avg_trans_upp[j*total_scenarios:j*total_scenarios+total_scenarios]),
                     color='green', alpha=0.5)
    # mc
    plt.plot(ind[j * total_scenarios:j * total_scenarios + total_scenarios],
             np.array(avg_trans_mc[j * total_scenarios:j * total_scenarios + total_scenarios]),
             color='red')
    plt.fill_between(ind[j * total_scenarios:j * total_scenarios + total_scenarios],
                     np.array(avg_trans_low_mc[j * total_scenarios:j * total_scenarios + total_scenarios]),
                     np.array(avg_trans_upp_mc[j * total_scenarios:j * total_scenarios + total_scenarios]),
                     color='red', alpha=0.5)

# plot the raw filtered average rmse data
for i in ind:
    x_values = [i]*len(scatter_trans[i])
    x_values_mc = [i]*len(scatter_trans_mc[i])
    plt.scatter(np.array(x_values), np.array(scatter_trans[i]), color='green', marker='x')
    plt.scatter(np.array(x_values_mc), np.array(scatter_trans_mc[i]), color='red', marker='x')

plt.plot([], [], color='green', label="ORB SLAM")
plt.plot([], [], color='red', label="ORB MC mode: no map point cullling")
plt.legend(loc='upper left', fontsize=f_legend)

for i in range(1, total_SL):
    plt.axvline(x=total_scenarios*i-0.5, color='black', alpha=0.5)

plt.grid(True, linestyle='--', alpha=0.5)
plt.title("Translational component root mean square error of the relative pose error", fontsize=fs_title, fontweight='bold')
plt.ylabel("Trans. RMSE RPE [m/m]", fontsize=fs_y)
plt.xticks(ind, x, fontsize=fs_x)
plt.yticks(fontsize=fs_x)
plt.xlim(left=x_lim_left, right=x_lim_right)
plt.subplots_adjust(hspace=0.5)

plt.subplot(3, 1, 2)
for j in range(total_SL):
    plt.plot(ind[j*total_scenarios:j*total_scenarios+total_scenarios],
             np.array(avg_rot[j*total_scenarios:j*total_scenarios+total_scenarios]), color='green')
    plt.fill_between(ind[j*total_scenarios:j*total_scenarios+total_scenarios],
                     np.array(avg_rot_low[j*total_scenarios:j*total_scenarios+total_scenarios]),
                     np.array(avg_rot_upp[j*total_scenarios:j*total_scenarios+total_scenarios]),
                     color='green', alpha=0.5)

    # mc
    plt.plot(ind[j*total_scenarios:j*total_scenarios+total_scenarios],
             np.array(avg_rot_mc[j*total_scenarios:j*total_scenarios+total_scenarios]), color='red')
    plt.fill_between(ind[j*total_scenarios:j*total_scenarios+total_scenarios],
                     np.array(avg_rot_low_mc[j*total_scenarios:j*total_scenarios+total_scenarios]),
                     np.array(avg_rot_upp_mc[j*total_scenarios:j*total_scenarios+total_scenarios]),
                     color='red', alpha=0.5)
for i in ind:
    x_values = [i]*len(scatter_trans[i])
    plt.scatter(np.array(x_values), np.array(scatter_rot[i]), color='green', marker='x')
    # mc
    x_values_mc = [i] * len(scatter_trans_mc[i])
    plt.scatter(np.array(x_values_mc), np.array(scatter_rot_mc[i]), color='red', marker='x')

plt.plot([], [], color='green', label="Average RMSE value")
plt.fill_between([], [], [], color="green", alpha=0.5, label="95% confidence interval")
plt.scatter([], [], color='green', marker='x', label="Raw RMSE values")

plt.legend(loc='upper left', fontsize=f_legend)

for i in range(1, total_SL):
    plt.axvline(x=total_scenarios*i-0.5, color='black', alpha=0.5)
plt.grid(True, linestyle='--', alpha=0.5)
plt.title("Rotational component root mean square error of the relative pose error", fontsize=fs_title, fontweight='bold')
plt.ylabel("Rot. RMSE RPE [deg/m]", fontsize=fs_y)
plt.xticks(ind, x, fontsize=fs_x)
plt.yticks(fontsize=fs_x)
plt.xlim(left=x_lim_left, right=x_lim_right)

plt.subplot(3, 1, 3)
width = 0.35
completed_track = np.array([100.0]*len(track_fail)) - np.array(track_fail) - np.array(loop_fail)
sum_bars = completed_track + np.array(loop_fail).tolist()

completed_track_mc = np.array([100.0]*len(track_fail_mc)) - np.array(track_fail_mc) - np.array(loop_fail_mc)
sum_bars_mc = completed_track_mc + np.array(loop_fail_mc).tolist()

ind_left = ind - width/2
ind_right = ind + width/2
plot_complete = plt.bar(ind_left, completed_track, width, color='green', alpha=0.4)
plot_loop = plt.bar(ind_left, loop_fail, width, bottom=completed_track, color='green', alpha=0.6, hatch='/')
plot_track = plt.bar(ind_left, track_fail, width, bottom=sum_bars, color='green', alpha=1.0, hatch='\\')

plot_complete_mc = plt.bar(ind_right, completed_track_mc, width, color='red', alpha=0.4)
plot_loop_mc = plt.bar(ind_right, loop_fail_mc, width, bottom=completed_track_mc, color='red', alpha=1.0, hatch='\\')
plot_track_mc = plt.bar(ind_right, track_fail_mc, width, bottom=sum_bars_mc, color='red', alpha=1.0, hatch='\\')

for i in range(1, total_SL):
    plt.axvline(x=total_scenarios*i-0.5, color='black', alpha=0.5)
plt.xticks(ind, x, fontsize=fs_x)
plt.yticks(fontsize=fs_x)
plt.ylabel("Data used and filtered [%]", fontsize=fs_y)
plt.title("Division successful and failed localization", fontsize=fs_title, fontweight='bold')
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend((plot_track, plot_loop, plot_complete), ("tracking failure", "false relocalization", "successful tracking"), loc='upper left', fontsize=f_legend)
plt.ylim(top=110.0)
plt.xlim(left=x_lim_left, right=x_lim_right)

plt.show()