from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir, FilterOutliersFromOrb
from matplotlib import pyplot as plt
import numpy as np
import pdb


def main():
    """Test if error bars show the results instinctively"""
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dynamic_scenarios = (20, 15)
    Towns = (1,2)

    # for Town in Towns:
    Town = 3
    if Town == 1:
        starting_locations = (0, 27, 58)
    elif Town == 2:
        starting_locations = (18, 37, 78)
    elif Town == 3:
        starting_locations = (75, 97, 127, 132)
    else:
        print("Town does not exist")
        return

    plt.figure("RMSE RPE trans")
    plt.title("RMSE Relative Pose Error translational component")
    # total number of entries
    N = (len(dynamic_scenarios)+1) * len(starting_locations)
    i = 0
    ind = np.arange(N)
    x = []

    for index, SL in enumerate(starting_locations):
        avg_trans = []
        std_trans = []
        avg_rot = []
        std_rot = []
        track_fail = []
        loop_fail = []

        for ds in dynamic_scenarios:
            dir_name_stat = DirName(Town, SL, "static")
            dir_name_dyn = DirName(Town, SL, "dynamic", ds)

            orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat)
            orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)
            # orb_static, orb_dynamic = FilterOutliersFromOrb(orb_static, orb_dynamic, Town, SL, ds)
            Perf = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
            if ds == 20:
                x.extend(["T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + " Static",
                          "T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + " Dist:" + str(Perf.scenario["Distance"])])
                avg_trans.extend([Perf.rmse_static_avg[0], Perf.rmse_dynamic_avg[0]])
                std_trans.extend([Perf.rmse_static_std[0], Perf.rmse_dynamic_std[0]])
                avg_rot.extend([Perf.rmse_static_avg[1], Perf.rmse_dynamic_avg[1]])
                std_rot.extend([Perf.rmse_static_std[1], Perf.rmse_dynamic_std[1]])
                track_fail.extend([Perf.lost_track_static, Perf.lost_track_dynamic])
                loop_fail.extend([Perf.false_loop_static, Perf.false_loop_dynamic])

            else:
                x.append("T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + " Dist:" + str(Perf.scenario["Distance"]))
                avg_trans.append(Perf.rmse_dynamic_avg[0])
                std_trans.append(Perf.rmse_dynamic_std[0])
                avg_rot.append(Perf.rmse_dynamic_avg[1])
                std_rot.append(Perf.rmse_dynamic_std[1])
                track_fail.append(Perf.lost_track_dynamic)
                loop_fail.append(Perf.false_loop_dynamic)
        # have to loop it otherwise you get the same color
        plt.subplot(3, 1, 1)
        # plt.grid(True)
        label_location = "T" + str(Perf.location["Town"]) + "Nr" + str(index+1)
        plt.errorbar(ind[i:i+(len(dynamic_scenarios)+1)], np.array(avg_trans), np.array(std_trans), label=label_location)
        plt.legend()

        plt.subplot(3, 1, 2)
        label_location = "T" + str(Perf.location["Town"]) + "Nr" + str(index + 1)
        plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot), np.array(std_rot), label=label_location)
        plt.legend()

        width = 0.35
        plt.subplot(3, 1, 3)
        plot_track = plt.bar(ind[i:i+(len(dynamic_scenarios)+1)], track_fail, width, color="r")
        plot_loop = plt.bar(ind[i:i+(len(dynamic_scenarios)+1)], loop_fail, width, bottom=track_fail, color="b")
        i = i + (len(dynamic_scenarios)+1)
    plt.subplot(3, 1, 1)
    plt.xticks(ind, x)
    plt.xlim(right=N)
    plt.title("Root Mean Square Error Relative Pose Error translational component")

    plt.subplot(3, 1, 2)
    plt.xticks(ind, x)
    plt.xlim(right=N)
    plt.title("Root Mean Square Error Relative Pose Error rotational component")

    plt.subplot(3, 1, 3)
    plt.title("Ratio removed due to false loop closure or tracking failure")
    plt.xticks(ind,x)
    plt.ylim(top=1.1)
    plt.xlim(right=N)
    plt.legend((plot_track, plot_loop), ("tracking failure", "false loop closure"))
    plt.show()


if __name__ == "__main__":
    main()
