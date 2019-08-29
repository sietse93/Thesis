from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb


def main():
    """Test if error bars show the results instinctively"""
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dynamic_scenarios = (20, 15, 10)
    Towns = (1,2)
    mode = "SLAM"

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
    i = 0  # index for the error plots
    j = 0  # index for the bar plots
    ind = np.arange(N)
    ind_bar = np.arange(3*N)  # for each trajectory we want 3 representations
    x = []

    for index, SL in enumerate(starting_locations):
        avg_trans = []
        std_trans = []
        avg_rot = []
        std_rot = []
        track_fail = []
        loop_fail = []

        avg_trans_vo = []
        std_trans_vo = []
        avg_rot_vo = []
        std_rot_vo = []
        track_fail_vo = []
        loop_fail_vo = []

        avg_trans_mc = []
        std_trans_mc = []
        avg_rot_mc = []
        std_rot_mc = []
        track_fail_mc = []
        loop_fail_mc = []

        for ds in dynamic_scenarios:
            dir_name_stat = DirName(Town, SL, "static")
            dir_name_dyn = DirName(Town, SL, "dynamic", ds)

            # SLAM performance
            orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "SLAM")
            orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "SLAM")

            # VO performance
            orb_static_vo, gt_vo = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "VO")
            orb_dynamic_vo, gt_vo = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "VO")

            # MC performance
            orb_static_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "MC")
            orb_dynamic_mc, gt_mc = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "MC")

            Perf = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
            Perf_vo = ScenarioLocationPerformance(ds, Town, SL, orb_static_vo, orb_dynamic_vo, gt_vo)
            Perf_mc = ScenarioLocationPerformance(ds, Town, SL, orb_static_mc, orb_dynamic_mc, gt_mc)

            if ds == 20:
                x.extend(["T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + "\n" + " Static",
                          "T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + "\n" + " Dist:" + str(Perf.scenario["Distance"])])
                # get SLAM data
                avg_trans.extend([Perf.rmse_static_avg[0], Perf.rmse_dynamic_avg[0]])
                std_trans.extend([Perf.rmse_static_std[0], Perf.rmse_dynamic_std[0]])
                avg_rot.extend([Perf.rmse_static_avg[1], Perf.rmse_dynamic_avg[1]])
                std_rot.extend([Perf.rmse_static_std[1], Perf.rmse_dynamic_std[1]])
                track_fail.extend([Perf.lost_track_static, Perf.lost_track_dynamic])
                loop_fail.extend([Perf.false_loop_static, Perf.false_loop_dynamic])

                # get VO data
                avg_trans_vo.extend([Perf_vo.rmse_static_avg[0], Perf_vo.rmse_dynamic_avg[0]])
                std_trans_vo.extend([Perf_vo.rmse_static_std[0], Perf_vo.rmse_dynamic_std[0]])
                avg_rot_vo.extend([Perf_vo.rmse_static_avg[1], Perf_vo.rmse_dynamic_avg[1]])
                std_rot_vo.extend([Perf_vo.rmse_static_std[1], Perf_vo.rmse_dynamic_std[1]])
                track_fail_vo.extend([Perf_vo.lost_track_static, Perf_vo.lost_track_dynamic])
                loop_fail_vo.extend([Perf_vo.false_loop_static, Perf_vo.false_loop_dynamic])

                # get MC data
                avg_trans_mc.extend([Perf_mc.rmse_static_avg[0], Perf_mc.rmse_dynamic_avg[0]])
                std_trans_mc.extend([Perf_mc.rmse_static_std[0], Perf_mc.rmse_dynamic_std[0]])
                avg_rot_mc.extend([Perf_mc.rmse_static_avg[1], Perf_mc.rmse_dynamic_avg[1]])
                std_rot_mc.extend([Perf_mc.rmse_static_std[1], Perf_mc.rmse_dynamic_std[1]])
                track_fail_mc.extend([Perf_mc.lost_track_static, Perf_mc.lost_track_dynamic])
                loop_fail_mc.extend([Perf_mc.false_loop_static, Perf_mc.false_loop_dynamic])
            else:
                x.append("T" + str(Perf.location["Town"]) + "Nr" + str(index+1) + "\n" + "Dist:" + str(Perf.scenario["Distance"]))

                # get SLAM data
                avg_trans.append(Perf.rmse_dynamic_avg[0])
                std_trans.append(Perf.rmse_dynamic_std[0])
                avg_rot.append(Perf.rmse_dynamic_avg[1])
                std_rot.append(Perf.rmse_dynamic_std[1])
                track_fail.append(Perf.lost_track_dynamic)
                loop_fail.append(Perf.false_loop_dynamic)

                # get VO data
                avg_trans_vo.append(Perf_vo.rmse_dynamic_avg[0])
                std_trans_vo.append(Perf_vo.rmse_dynamic_std[0])
                avg_rot_vo.append(Perf_vo.rmse_dynamic_avg[1])
                std_rot_vo.append(Perf_vo.rmse_dynamic_std[1])
                track_fail_vo.append(Perf_vo.lost_track_dynamic)
                loop_fail_vo.append(Perf_vo.false_loop_dynamic)

                # get MC data
                avg_trans_mc.append(Perf_mc.rmse_dynamic_avg[0])
                std_trans_mc.append(Perf_mc.rmse_dynamic_std[0])
                avg_rot_mc.append(Perf_mc.rmse_dynamic_avg[1])
                std_rot_mc.append(Perf_mc.rmse_dynamic_std[1])
                track_fail_mc.append(Perf_mc.lost_track_dynamic)
                loop_fail_mc.append(Perf_mc.false_loop_dynamic)
        # have to loop it otherwise you get the same color
        plt.subplot(3, 1, 1)
        # plt.grid(True)
        plot_vo_trans = plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans_vo), np.array(std_trans_vo), color='blue',
                     label="VO")

        plot_mc_trans = plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans_mc), np.array(std_trans_mc),
                     color='red', label="MC")
        plot_slam_trans = plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans), np.array(std_trans), color='lime',
                     label="SLAM")

        plt.subplots_adjust(hspace=0.5)

        plt.subplot(3, 1, 2)
        plot_vo_rot = plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_vo), np.array(std_rot_vo), color='blue',
                     label="VO")

        plot_mc_rot = plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_mc), np.array(std_rot_mc), color='red',
                     label="MC")
        plot_slam_rot = plt.errorbar(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot), np.array(std_rot), color='lime',
                     label="SLAM")

        width = 0.3
        tf_color = "black"
        lc_color = "silver"

        plt.subplot(3, 1, 3)
        plt.bar(ind_bar[j:j+3*4:3], track_fail, width, color=tf_color)
        plt.bar(ind_bar[j:j+3*4:3], loop_fail, width, bottom=track_fail, color=lc_color)
        plt.bar(ind_bar[j+1:j+1+3*4:3], track_fail_vo, width, color=tf_color)
        plt.bar(ind_bar[j+1:j+1+3*4:3], loop_fail_vo, width, bottom=track_fail_vo, color=lc_color)
        plot_tf = plt.bar(ind_bar[j + 2:j + 2 + 3 * 4:3], track_fail_mc, width, color=tf_color)
        plot_lc = plt.bar(ind_bar[j + 2:j + 2 + 3 * 4:3], loop_fail_mc, width, bottom=track_fail_mc, color=lc_color)
        # plot_track = plt.bar(ind_bar[j:j+(len(dynamic_scenarios)+1):3], track_fail, width, color="r")
        # plot_loop = plt.bar(ind_bar[j:j+(len(dynamic_scenarios)+1):3], loop_fail, width, bottom=track_fail, color="b")
        # pdb.set_trace()
        i = i + (len(dynamic_scenarios)+1)
        j = j + 3*(len(dynamic_scenarios)+1)


    plt.subplot(3, 1, 1)
    plt.xticks(ind, x)
    plt.xlim(right=N)
    plt.title("Translational component root mean square error of the relative pose error")
    plt.ylabel("Trans. RMSE RPE [-]")
    plt.legend([plot_slam_trans, plot_vo_trans, plot_mc_trans], ["SLAM", "VO", "MC"])

    plt.subplot(3, 1, 2)
    plt.xticks(ind, x)
    plt.xlim(right=N)
    plt.title("Rotational component root mean square error relative pose error")
    plt.ylabel("Rot. RMSE RPE [deg/m]")
    plt.legend([plot_slam_rot, plot_vo_rot, plot_mc_rot], ["SLAM", "VO", "MC"])
    plt.subplot(3, 1, 3)
    plt.title("Ratio removed due to false loop closure or tracking failure")
    bar_ticks = ["" for i in range(3*N)]
    for index, label in enumerate(x):
        new_index = index*3
        bar_ticks[new_index] = label + "\n SLAM"
        try:
            bar_ticks[new_index+1] = "VO"
            bar_ticks[new_index+2] = "MC"
        except IndexError:
            continue
    plt.xticks(ind_bar, bar_ticks)
    plt.legend([plot_tf, plot_lc], ["Tracking failure", "False relocalization"])
    # plt.xticks(ind_bar[0::3], x)
    plt.ylim(top=1.1)
    plt.xlim(right=3*N)
    plt.ylabel("Ratio removed of total data [-]")
    # plt.legend((plot_track, plot_loop), ("tracking failure", "false loop closure"))
    plt.show()


if __name__ == "__main__":
    main()
