from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb


def main():
    """Produces test error bars that compares ORB VO with and without map culling method"""
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dynamic_scenarios = (20, 15, 10)
    Towns = (1,2)
    mode = "MC"

    # for Town in Towns:
    Town = 2
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
        avg_trans_vo = []
        std_trans_vo = []
        avg_rot_vo = []
        std_rot_vo = []

        avg_trans_mc = []
        std_trans_mc = []
        avg_rot_mc = []
        std_rot_mc = []
        for ds in dynamic_scenarios:
            dir_name_stat = DirName(Town, SL, "static")
            dir_name_dyn = DirName(Town, SL, "dynamic", ds)
            orb_static_vo, gt_vo = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "VO")
            orb_dynamic_vo, gt_vo = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "VO")
            Perf_vo = ScenarioLocationPerformance(ds, Town, SL, orb_static_vo, orb_dynamic_vo, gt_vo)

            orb_static_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "MC")
            orb_dynamic_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "MC")
            Perf_mc = ScenarioLocationPerformance(ds, Town, SL, orb_static_mc, orb_dynamic_mc, gt_mc)
            if ds == 20:
                x.extend(["T" + str(Perf_vo.location["Town"]) + "Nr" + str(index+1) + "\n" +" Static",
                          "T" + str(Perf_vo.location["Town"]) + "Nr" + str(index+1) + "\n"+ " Dist:" + str(Perf_vo.scenario["Distance"])])
                avg_trans_vo.extend([Perf_vo.rmse_static_avg[0], Perf_vo.rmse_dynamic_avg[0]])
                std_trans_vo.extend([Perf_vo.rmse_static_std[0], Perf_vo.rmse_dynamic_std[0]])
                avg_rot_vo.extend([Perf_vo.rmse_static_avg[1], Perf_vo.rmse_dynamic_avg[1]])
                std_rot_vo.extend([Perf_vo.rmse_static_std[1], Perf_vo.rmse_dynamic_std[1]])

                avg_trans_mc.extend([Perf_mc.rmse_static_avg[0], Perf_mc.rmse_dynamic_avg[0]])
                std_trans_mc.extend([Perf_mc.rmse_static_std[0], Perf_mc.rmse_dynamic_std[0]])
                avg_rot_mc.extend([Perf_mc.rmse_static_avg[1], Perf_mc.rmse_dynamic_avg[1]])
                std_rot_mc.extend([Perf_mc.rmse_static_std[1], Perf_mc.rmse_dynamic_std[1]])

            else:
                x.append("T" + str(Perf_vo.location["Town"]) + "Nr" + str(index+1) + "\n" + "Dist:" + str(Perf_vo.scenario["Distance"]))
                avg_trans_vo.append(Perf_vo.rmse_dynamic_avg[0])
                std_trans_vo.append(Perf_vo.rmse_dynamic_std[0])
                avg_rot_vo.append(Perf_vo.rmse_dynamic_avg[1])
                std_rot_vo.append(Perf_vo.rmse_dynamic_std[1])

                avg_trans_mc.append(Perf_mc.rmse_dynamic_avg[0])
                std_trans_mc.append(Perf_mc.rmse_dynamic_std[0])
                avg_rot_mc.append(Perf_mc.rmse_dynamic_avg[1])
                std_rot_mc.append(Perf_mc.rmse_dynamic_std[1])

        print("avg trans vo", avg_trans_vo)
        print("avg trans mc", avg_trans_mc)

        print("avg rot vo", avg_rot_vo)
        print("avg rot mc", avg_rot_mc)
        # have to loop it otherwise you get the same color
        plt.subplot(2, 1, 1)
        print(i)
        # plt.grid(True)

        if i == (N-(len(dynamic_scenarios)+1)):
            plt.errorbar(ind[i:i+(len(dynamic_scenarios)+1)], np.array(avg_trans_vo), np.array(std_trans_vo), color='blue', label="ORB VO")
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans_mc), np.array(std_trans_mc), color='red', label="ORB VO, no map culling")
            plt.legend()
        else:
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans_vo), np.array(std_trans_vo),
                         color='blue')
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans_mc), np.array(std_trans_mc),
                         color='red')


        plt.subplots_adjust(hspace=0.5)

        plt.subplot(2, 1, 2)
        if i == (N-(len(dynamic_scenarios)+1)):
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_vo), np.array(std_rot_vo), color='blue', label="ORB VO")
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_mc), np.array(std_rot_mc), color='red', label="ORB VO, no map culling")
            plt.legend()
        else:
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_vo), np.array(std_rot_vo),
                         color='blue')
            plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_mc), np.array(std_rot_mc),
                         color='red')

        i = i + (len(dynamic_scenarios)+1)
    pdb.set_trace()
    plt.subplot(2, 1, 1)
    plt.xticks(ind, x, fontsize=14)
    if mode == "SLAM" and Town == 1:
        plt.ylim(top=0.060)
    plt.xlim(right=N)
    plt.title("Translational component root mean square error of the relative pose error", fontsize=18)
    plt.ylabel("Trans. RMSE RPE [-]", fontsize=14)

    plt.subplot(2, 1, 2)
    plt.xticks(ind, x, fontsize=14)
    if mode == "SLAM" and Town == 1:
        plt.ylim(top=0.040)
    plt.xlim(right=N)
    plt.title("Rotational component root mean square error relative pose error", fontsize=18)
    plt.ylabel("Rot. RMSE RPE [deg/m]", fontsize=14)
    plt.show()


if __name__ == "__main__":
    main()
