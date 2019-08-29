from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb


def main():
    """Test if error bars show the results instinctively"""
    base_dir_dyn = "/home/sietse/results_carla0.9/VansOppositeRoad/"
    base_dir_stat = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    # dynamic variable (10 cars on opposite lane)
    dv = 10
    Towns = (1, 2)
    mode = "SLAM"

    N = 2 * 6 
    i = 0
    ind = np.arange(N)
    x = []
    for Town in Towns:
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
        # total number of entries: static and non static

        for index, SL in enumerate(starting_locations):
            avg_trans = []
            std_trans = []
            avg_rot = []
            std_rot = []
            track_fail = []
            loop_fail = []

            avg_trans_mc = []
            std_trans_mc = []
            avg_rot_mc = []
            std_rot_mc = []
            track_fail_mc = []
            loop_fail_mc = []

            dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
            dir_name_dyn = "T{}_SL{}_d{}/".format(Town, SL, dv)

            # get the static data from the stuck behind van experiment
            orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir_stat, dir_name_stat, "SLAM")
            orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir_dyn, dir_name_dyn, "SLAM")

            orb_static_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir_stat, dir_name_stat, "MC")
            orb_dynamic_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir_dyn, dir_name_dyn, "MC")

            Perf_mc = ScenarioLocationPerformance(dv, Town, SL, orb_static_mc, orb_dynamic_mc, gt_mc)

            Perf = ScenarioLocationPerformance(dv, Town, SL, orb_static, orb_dynamic, gt)

            # first get the static data from that location
            x.append("T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + "\n" + " Static")
            avg_trans.append(Perf.rmse_static_avg[0])
            std_trans.append(Perf.rmse_static_std[0])
            avg_rot.append(Perf.rmse_static_avg[1])
            std_rot.append(Perf.rmse_static_std[1])
            track_fail.append(Perf.lost_track_static)
            loop_fail.append(Perf.false_loop_static)

            avg_trans_mc.append(Perf_mc.rmse_static_avg[0])
            std_trans_mc.append(Perf_mc.rmse_static_std[0])
            avg_rot_mc.append(Perf_mc.rmse_static_avg[1])
            std_rot_mc.append(Perf_mc.rmse_static_std[1])
            track_fail_mc.append(Perf_mc.lost_track_static)
            loop_fail_mc.append(Perf_mc.false_loop_static)

            x.append("T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + "\n" + "Dynamic")
            avg_trans.append(Perf.rmse_dynamic_avg[0])
            std_trans.append(Perf.rmse_dynamic_std[0])
            avg_rot.append(Perf.rmse_dynamic_avg[1])
            std_rot.append(Perf.rmse_dynamic_std[1])
            track_fail.append(Perf.lost_track_dynamic)
            loop_fail.append(Perf.false_loop_dynamic)

            avg_trans_mc.append(Perf_mc.rmse_dynamic_avg[0])
            std_trans_mc.append(Perf_mc.rmse_dynamic_std[0])
            avg_rot_mc.append(Perf_mc.rmse_dynamic_avg[1])
            std_rot_mc.append(Perf_mc.rmse_dynamic_std[1])
            track_fail_mc.append(Perf_mc.lost_track_dynamic)
            loop_fail_mc.append(Perf_mc.false_loop_dynamic)

            # have to loop it otherwise you get the same color
            plt.subplot(3, 1, 1)
            # plt.grid(True)
            label_location = "T" + str(Perf.location["Town"]) + "Nr" + str(index+1)
            if i == N-2:
                plt.errorbar(ind[i:i+2], np.array(avg_trans), np.array(std_trans), color='green', label="ORB SLAM")
                plt.errorbar(ind[i:i + 2], np.array(avg_trans_mc), np.array(std_trans_mc), color='red', label="ORB VO No map culling")
                plt.legend()
            else:
                plt.errorbar(ind[i:i + 2], np.array(avg_trans), np.array(std_trans), color='green')
                plt.errorbar(ind[i:i + 2], np.array(avg_trans_mc), np.array(std_trans_mc), color='red')
            plt.subplots_adjust(hspace=0.5)

            plt.subplot(3, 1, 2)
            label_location = "T" + str(Perf.location["Town"]) + "Nr" + str(index + 1)
            if i == N-2:
                plt.errorbar(ind[i:i+2], np.array(avg_rot), np.array(std_rot), color='green', label="ORB SLAM")
                plt.errorbar(ind[i:i + 2], np.array(avg_rot_mc), np.array(std_rot_mc), color='red', label="ORB VO No map culling")
                plt.legend()
            else:
                plt.errorbar(ind[i:i + 2], np.array(avg_rot), np.array(std_rot), color='green')
                plt.errorbar(ind[i:i + 2], np.array(avg_rot_mc), np.array(std_rot_mc), color='red')


            width = 0.35
            plt.subplot(3, 1, 3)
            plot_track = plt.bar(ind[i:i+2], track_fail, width, color="r")
            plot_loop = plt.bar(ind[i:i+2], loop_fail, width, bottom=track_fail, color="b")
            i = i + 2
    plt.subplot(3, 1, 1)
    plt.xticks(ind, x)
    plt.xlim(right=N)
    plt.title("Translational component root mean square error of the relative pose error")
    plt.ylabel("Trans. RMSE RPE [-]")

    plt.subplot(3, 1, 2)
    plt.xticks(ind, x)
    plt.xlim(right=N)
    plt.title("Rotational component root mean square error relative pose error")
    plt.ylabel("Rot. RMSE RPE [deg/m]")
    plt.subplot(3, 1, 3)
    plt.title("Ratio removed due to false loop closure or tracking failure")
    plt.xticks(ind,x)
    plt.ylim(top=1.1)
    plt.xlim(right=N)
    plt.ylabel("Ratio removed of total data [-]")
    plt.legend((plot_track, plot_loop), ("tracking failure", "false loop closure"))
    plt.show()


if __name__ == "__main__":
    main()
