import json
from func_Convert2Json import json2crf
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import numpy as np
import pdb


def main():
    # This file will plot the results of the pose estimation in static conditions with a
    # predetermined map that is made under various circumstances.
    base_dir = "/home/sietse/results_carla0.9/"
    town = 1

    starting_locations = (0, 27, 58)

    # quick check of all localization and mapping results to see if sc2 is the least accurate scenario

    sc1_avg_rmse_trans, sc1_std_rmse_trans, sc1_avg_rmse_rot, sc1_std_rmse_rot = \
        AvgStdRmseMapLoc(base_dir, town, starting_locations, "Sc1")

    sc2_avg_rmse_trans, sc2_std_rmse_trans, sc2_avg_rmse_rot, sc2_std_rmse_rot = \
        AvgStdRmseMapLoc(base_dir, town, starting_locations, "Sc2")

    pose_estimation = "Stat"

    maploc_avg_rmse_trans, maploc_std_rmse_trans, maploc_avg_rmse_rot, maploc_std_rmse_rot = \
        AvgStdRmseMapLoc(base_dir, town, starting_locations, pose_estimation)

    statstat_avg_rmse_trans, statstat_std_rmse_trans, statstat_avg_rmse_rot, statstat_std_rmse_rot = \
        AvgStdRmseNoMapping(base_dir, town, starting_locations, "Stat", pose_estimation)

    sc2stat_avg_rmse_trans, sc2stat_std_rmse_trans, sc2stat_avg_rmse_rot, sc2stat_std_rmse_rot = \
        AvgStdRmseNoMapping(base_dir, town, starting_locations, "Sc2", pose_estimation)

    # sc1stat_avg_rmse_trans, sc1stat_std_rmse_trans, sc1stat_avg_rmse_rot, sc1stat_std_rmse_rot = \
    #     AvgStdRmseNoMapping(base_dir, town, starting_locations, "Sc1", pose_estimation)

    ind = np.arange(len(starting_locations))  # each trajectory
    width = 0.2


    # sets the size axis labels
    fs_x = 14
    # sets the size of the texts
    fs_y = 16
    # set size of legend
    f_legend = 12
    fs_title = 16

    ## Quick check to compare general behavior between the three scenarios
    plt.figure("Pose estimation: Localization and mapping Town 1")
    plt.suptitle("Results town 1 ORB SLAM", fontsize=18, fontweight='bold', y=0.95)
    plt.subplot(2, 1, 1)
    plt.title("Translational component root mean square error of the relative pose error", fontsize=fs_title,
              fontweight='bold')
    ind_x = [0.2, 1.2, 2.2]
    strings_x = ["Town 1 - trajectory 1", "Town 1 - trajectory 2", "Town 1 - trajectory 3"]
    plt.bar(ind, np.array(maploc_avg_rmse_trans), width, yerr=2 * np.array(maploc_std_rmse_trans), color='sienna')
    plt.bar(ind + 1 * width, np.array(sc1_avg_rmse_trans), width, yerr=2 * np.array(sc1_std_rmse_trans),
            color='gold')
    plt.bar(ind + 2*width, np.array(sc2_avg_rmse_trans), width, yerr=2 * np.array(sc2_std_rmse_trans),
            color='olivedrab')
    plt.xticks(ind_x, strings_x, fontsize=fs_x)
    plt.ylabel("Trans. RMSE RPE [m/m]", fontsize=fs_y)
    plt.yticks(fontsize=fs_x)
    plt.grid(True, linestyle='--', alpha=0.5)

    plt.subplot(2, 1, 2)
    plt.title("Rotational component root mean square error of the relative pose error", fontsize=fs_title,
              fontweight='bold')
    plt.bar(ind, np.array(maploc_avg_rmse_rot), width, yerr=2 * np.array(maploc_std_rmse_rot), color='sienna')
    plt.bar(ind + 1 * width, np.array(sc1_avg_rmse_rot), width, yerr=2 * np.array(sc1_std_rmse_rot),
            color='gold')
    plt.bar(ind + 2*width, np.array(sc2_avg_rmse_rot), width, yerr=2 * np.array(sc2_std_rmse_rot),
            color='olivedrab')

    plt.xticks(ind_x, strings_x, fontsize=fs_x)

    plt.ylabel("Trans. RMSE RPE [m/m]", fontsize=fs_y)
    plt.yticks(fontsize=fs_x)
    plt.grid(True, linestyle='--', alpha=0.5)

    legend_static = mpatches.Patch(color='sienna', label="ORB SLAM: static")
    legend_sc1 = mpatches.Patch(color='gold', label="ORB SLAM: vehicle in front")
    legend_sc2 = mpatches.Patch(color='olivedrab', label="ORB SLAM: vehicles opposite lane")
    legend_std = mlines.Line2D([], [], color='black', linestyle='-', label="95% confidence interval")

    plt.legend(handles=[legend_static, legend_sc1, legend_sc2, legend_std], fontsize=f_legend)

    plt.figure("Pose estimation: Static conditions with predetermined map")
    plt.suptitle("Results town 1 static environment: utilize map created various conditions", fontsize=18, fontweight='bold', y=0.95)
    plt.subplot(2, 1, 1)
    plt.title("Translational component root mean square error of the relative pose error", fontsize=fs_title, fontweight='bold')
    plt.bar(ind, np.array(maploc_avg_rmse_trans), width, yerr=2*np.array(maploc_std_rmse_trans), color='black', label="Static: simultaneous localization and mapping")
    plt.bar(ind+1*width, np.array(statstat_avg_rmse_trans), width,  yerr=2*np.array(statstat_std_rmse_trans), color='sienna', label="Pose = static, map = static")
    plt.bar(ind + 2 * width, np.array(sc1stat_avg_rmse_trans), width, yerr=2 * np.array(sc1stat_std_rmse_trans),
            color='gold', label="Pose = static, map = vehicle in front")
    plt.bar(ind + 3*width, np.array(sc2stat_avg_rmse_trans), width, yerr=2 * np.array(sc2stat_std_rmse_trans),
            color='olivedrab', label="Pose = static, map = vehicles opposite lane")

    ind_x = [0.3, 1.3, 2.3]
    strings_x = ["Town 1 - trajectory 1", "Town 1 - trajectory 2", "Town 1 trajectory - 3"]
    plt.xticks(ind_x, strings_x, fontsize=fs_x)

    plt.ylabel("Trans. RMSE RPE [m/m]", fontsize=fs_y)
    plt.yticks(fontsize=fs_x)
    plt.grid(True, linestyle='--', alpha=0.5)


    plt.subplot(2, 1, 2)
    plt.title("Rotational component root mean square error of the relative pose error", fontsize=fs_title, fontweight='bold')
    plt.bar(ind, np.array(maploc_avg_rmse_rot), width, yerr=2 * np.array(maploc_std_rmse_rot), color='black',
            label="Static: simultaneous localization and mapping")
    plt.bar(ind + 1 * width, np.array(statstat_avg_rmse_rot), width, yerr=2 * np.array(statstat_std_rmse_rot),
            color='sienna', label="Pose = static, map = static")
    plt.bar(ind + 2 * width, np.array(sc1stat_avg_rmse_rot), width, yerr=2 * np.array(sc1stat_std_rmse_rot),
            color='gold', label="Pose = static, map = vehicle in front")
    plt.bar(ind + 3 * width, np.array(sc2stat_avg_rmse_rot), width, yerr=2 * np.array(sc2stat_std_rmse_rot),
            color='olivedrab', label="Pose = static, map = vehicles opposite lane")
    plt.ylim(bottom=0)
    plt.ylabel("Rot. RMSE RPE [deg/m]", fontsize=fs_y)
    plt.xticks(ind_x, strings_x, fontsize=fs_x)
    plt.yticks(fontsize=fs_x)

    legend_slam = mpatches.Patch(color='black', label='"Static: simultaneous localization and mapping"')
    legend_static = mpatches.Patch(color='sienna', label="Pose = static, map = static")
    legend_sc1 = mpatches.Patch(color='gold', label="Pose = static, map = vehicle in front")
    legend_sc2 = mpatches.Patch(color='olivedrab', label="Pose = static, map = vehicles opposite lane")
    legend_std = mlines.Line2D([], [], color='black', linestyle='-', label="95% confidence interval")

    plt.legend(handles=[legend_slam, legend_static, legend_sc1, legend_sc2, legend_std], fontsize=f_legend)
    plt.grid(True, linestyle='--', alpha=0.5)


    plt.show()


def AvgStdRmseMapLoc(base_dir, town, starting_locations, pose_estimation):
    """Compare the result to localization without map """

    avg_rmse_trans_list = []
    std_rmse_trans_list = []

    avg_rmse_rot_list = []
    std_rmse_rot_list = []

    for SL in starting_locations:

        if pose_estimation == "Stat":
            folder = "stuckbehindvan/20fps/T{}_SL{}_s".format(town, SL)
            file_name = "/T{}_SL{}_s_orb_{}_json.txt"
        elif pose_estimation == "Sc1":
            folder = "stuckbehindvan/20fps/T{}_SL{}_d15".format(town, SL)
            file_name = "/T{}_SL{}_d15_orb_{}_json.txt"
        elif pose_estimation == "Sc2":
            folder = "VansOppositeRoad/T{}_SL{}_d10".format(town, SL)
            file_name = "/T{}_SL{}_d10_orb_{}_json.txt"

        dir = base_dir + folder

        # Collect the orb data and ground truth from json files
        orb_data = []

        for orb_nr in range(5):
            try:
                orb = json2crf(dir, file_name.format(town, SL, orb_nr))
                orb_data.append(orb)
            except IOError:
                continue

        # trajectory is always the same so ground truth is independent of scenario
        ground_truth_dir = base_dir + "stuckbehindvan/20fps/T{}_SL{}_s".format(town, SL)
        gt = json2crf(ground_truth_dir, "/T{}_SL{}_s_gt_json.txt".format(town, SL))

        rmse_trans_list = []
        rmse_rot_list = []

        for i, orb in enumerate(orb_data):

            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb, 100)
            trans_rmse = calc_rmse(trans_errors)
            rmse_trans_list.append(trans_rmse)

            rot_rmse = calc_rmse(rot_errors)
            rmse_rot_list.append(rot_rmse)

            if i == 4 and SL == 58 and pose_estimation == "Sc2":
                rmse_trans_list.pop(2)
                rmse_rot_list.pop(2)

        rmse_trans_all = np.array(rmse_trans_list)
        avg_rmse_trans = np.mean(rmse_trans_all)
        avg_rmse_trans_list.append(avg_rmse_trans)
        std_rmse_trans = np.std(rmse_trans_all)
        std_rmse_trans_list.append(std_rmse_trans)

        rmse_rot_all = np.array(rmse_rot_list)
        avg_rmse_rot = np.mean(rmse_rot_all)
        avg_rmse_rot_list.append(avg_rmse_rot)
        std_rmse_rot = np.std(rmse_rot_all)
        std_rmse_rot_list.append(std_rmse_rot)

    return avg_rmse_trans_list, std_rmse_trans_list, avg_rmse_rot_list, std_rmse_rot_list


def AvgStdRmseNoMapping(base_dir, town, starting_locations, map_estimation, pose_estimation):
    """Input which combination results you want to have (e.g. static map,  pose estimation sc 2) and starting locations.
    Function outputs the avg and std rmse values"""

    avg_rmse_trans_list = []
    std_rmse_trans_list = []

    avg_rmse_rot_list = []
    std_rmse_rot_list = []

    for SL in starting_locations:

        if pose_estimation == "Stat":
            folder = "stuckbehindvan/20fps/T{}_SL{}_s".format(town, SL)
        elif pose_estimation == "Sc1":
            folder = "stuckbehindvan/20fps/T{}_SL{}_d15".format(town, SL)
        elif pose_estimation == "Sc2":
            folder = "VansOppositeRoad/T{}_SL{}_d10".format(town, SL)

        string_convention = map_estimation + pose_estimation

        dir = base_dir + folder

        # Collect the orb data and ground truth from json files
        orb_data = []
        for orb_nr in range(5):
            file_name = "/T{}_SL{}_LM_{}_{}_json.txt".format(town, SL, string_convention, orb_nr)
            try:
                orb = json2crf(dir, file_name)
                orb_data.append(orb)
            except IOError:
                continue

        # False relocalization happened during mapping in one of the orb data points
        if SL == 0 and map_estimation == "Sc2":
            orb_data.pop(0)

        # trajectory is always the same so ground truth is independent of scenario
        ground_truth_dir = base_dir + "stuckbehindvan/20fps/T{}_SL{}_s".format(town, SL)
        gt = json2crf(ground_truth_dir, "/T{}_SL{}_s_gt_json.txt".format(town, SL))

        rmse_trans_list = []
        rmse_rot_list = []
        for i, orb in enumerate(orb_data):
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb, 100)

            # utilize data from a certain point in time, before that ORB SLAM may not realize that a new rosbag has started
            for index, time in enumerate(time_used):
                if time >= 5.0:
                    index_used = index
                    break

            time_used = time_used[index_used::]
            trans_errors = trans_errors[index_used::]
            rot_errors = rot_errors[index_used::]

            trans_rmse = calc_rmse(trans_errors)
            rmse_trans_list.append(trans_rmse)

            rot_rmse = calc_rmse(rot_errors)
            rmse_rot_list.append(rot_rmse)

        rmse_trans_all = np.array(rmse_trans_list)
        avg_rmse_trans = np.mean(rmse_trans_all)
        avg_rmse_trans_list.append(avg_rmse_trans)
        std_rmse_trans = np.std(rmse_trans_all)
        std_rmse_trans_list.append(std_rmse_trans)

        rmse_rot_all = np.array(rmse_rot_list)
        avg_rmse_rot = np.mean(rmse_rot_all)
        avg_rmse_rot_list.append(avg_rmse_rot)
        std_rmse_rot = np.std(rmse_rot_all)
        std_rmse_rot_list.append(std_rmse_rot)
        pdb.set_trace()
    return avg_rmse_trans_list, std_rmse_trans_list, avg_rmse_rot_list, std_rmse_rot_list


if __name__ == "__main__":
    main()