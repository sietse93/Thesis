from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb

# Goal is to get the data without the complicated loop structure.
# then we can get the scatter plots in there

base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
dynamic_scenarios = (20, 15, 10)
Towns = (1, 2)
mode = "SLAM"

avg_trans = []
avg_trans_upp = []
avg_trans_low = []
avg_rot = []
avg_rot_upp = []
avg_rot_low = []
track_fail = []
loop_fail = []
x = []

x_scatter = []
scatter_trans = []
scatter_rot = []

Town = 1

if Town == 1:
    starting_locations = (0, 27, 58)
elif Town == 2:
    starting_locations = (18, 37, 78)
elif Town == 3:
    starting_locations = (75, 97, 127, 132)
else:
    print("Town does not exist")


for index, SL in enumerate(starting_locations):

    for ds in dynamic_scenarios:
        dir_name_stat = DirName(Town, SL, "static")
        dir_name_dyn = DirName(Town, SL, "dynamic", ds)

        orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, mode)
        orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, mode)
        Perf = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)

        if ds == 20:
            x.extend(["T" + str(Perf.location["Town"]) + "Nr" + str(index + 1) + "\n" + " Static",
                      "T" + str(Perf.location["Town"]) + "Nr" + str(index + 1) + "\n" + " Dist:" + str(
                          Perf.scenario["Distance"])])
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

        else:
            x.append("T" + str(Perf.location["Town"]) + "Nr" + str(index + 1) + "\n" + "Dist:" + str(
                Perf.scenario["Distance"]))
            avg_trans.append(Perf.rmse_dynamic_avg[0])
            avg_trans_upp.append(Perf.rmse_dynamic_avg[0] + 2 * Perf.rmse_dynamic_std[0])
            avg_trans_low.append(Perf.rmse_dynamic_avg[0] - 2 * Perf.rmse_dynamic_std[0])

            raw_trans_dynamic = [raw_rmse_dynamic[0] for raw_rmse_dynamic in Perf.rmse_dynamic]
            scatter_trans.append(raw_trans_dynamic)

            avg_rot.append(Perf.rmse_dynamic_avg[1])
            avg_rot_upp.append(Perf.rmse_dynamic_avg[1] + 2 * Perf.rmse_dynamic_std[1])
            avg_rot_low.append(Perf.rmse_dynamic_avg[1] - 2 * Perf.rmse_dynamic_std[1])

            raw_rot_dynamic = [raw_rmse_dynamic[1] for raw_rmse_dynamic in Perf.rmse_dynamic]
            scatter_rot.append(raw_rot_dynamic)

            track_fail.append(Perf.lost_track_dynamic)
            loop_fail.append(Perf.false_loop_dynamic)

track_fail = [cell*100.0 for cell in track_fail]
loop_fail = [cell*100.0 for cell in loop_fail]


fs_x = 14
fs_y = 14
fs_title = 16
x_lim_right = len(x)-0.5

plt.figure("Performance SLAM Algorithm")
plt.suptitle("Results Town {}: ORB in SLAM mode".format(Town), fontsize=18, fontweight='bold', y=0.95)
ind = np.arange(len(x))
plt.subplot(3, 1, 1)
plt.plot(ind, np.array(avg_trans), color='green', label="Average RMSE Value")
plt.fill_between(ind, np.array(avg_trans_low), np.array(avg_trans_upp), color='green', alpha=0.5, label="95% confidence interval")


# plot the raw filtered average rmse data
for i in ind:
    x_values = [i]*len(scatter_trans[i])
    if i == ind[-1]:
        plt.scatter(np.array(x_values), np.array(scatter_trans[i]), color='green', marker='x', label="Raw RMSE values")
    else:
        plt.scatter(np.array(x_values), np.array(scatter_trans[i]), color='green', marker='x')


plt.legend()

# plot the vertical lines
scenario_block = len(dynamic_scenarios)+1.0
for i in range(1, len(starting_locations)):
    plt.axvline(x=scenario_block*i-0.5, color='black', alpha=0.5)

plt.grid(True, linestyle='--', alpha=0.5)
plt.title("Translational component root mean square error of the relative pose error", fontsize=fs_title, fontweight='bold')
plt.ylabel("Trans. RMSE RPE [m/m]", fontsize=fs_y)
plt.xticks(ind, x, fontsize=fs_x)
plt.xlim(right=x_lim_right)
plt.subplots_adjust(hspace=0.5)

plt.subplot(3, 1, 2)
plt.plot(ind, np.array(avg_rot), color='green')
plt.fill_between(ind, np.array(avg_rot_low), np.array(avg_rot_upp), color='green', alpha=0.5)
for i in ind:
    x_values = [i]*len(scatter_trans[i])
    plt.scatter(np.array(x_values), np.array(scatter_rot[i]), color='green', marker='x')

# plot the vertical lines
scenario_block = len(dynamic_scenarios)+1.0
for i in range(1, len(starting_locations)):
    plt.axvline(x=scenario_block*i-0.5, color='black', alpha=0.5)
plt.grid(True, linestyle='--', alpha=0.5)
plt.title("Rotational component root mean square error of the relative pose error", fontsize=fs_title, fontweight='bold')
plt.ylabel("Rot. RMSE RPE [deg/m]", fontsize=fs_y)
plt.xticks(ind, x, fontsize=fs_x)
plt.xlim(right=x_lim_right)

plt.subplot(3, 1, 3)
width = 0.35
completed_track = np.array([100.0]*len(track_fail)) - np.array(track_fail) - np.array(loop_fail)
sum_bars = completed_track + np.array(loop_fail).tolist()
plot_complete = plt.bar(ind, completed_track, width, color='green', alpha=0.4)
plot_loop = plt.bar(ind, loop_fail, width, bottom=completed_track, color='green', alpha=0.6, hatch='/')
plot_track = plt.bar(ind, track_fail, width, bottom=sum_bars, color='green', alpha=1.0, hatch='\\')

# plot the vertical lines
scenario_block = len(dynamic_scenarios)+1.0
for i in range(1, len(starting_locations)):
    plt.axvline(x=scenario_block*i-0.5, color='black', alpha=0.5)
plt.xticks(ind, x, fontsize=fs_x)
plt.ylabel("Data used and filtered [%]", fontsize=fs_y)
plt.title("Division successful and failed localization", fontsize=fs_title, fontweight='bold')
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend((plot_track, plot_loop, plot_complete), ("tracking failure", "false relocalization", "successful tracking"), loc='upper right')
plt.ylim(top=110.0)
plt.xlim(right=x_lim_right)

plt.show()