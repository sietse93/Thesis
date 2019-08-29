from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb

def main():
    """Use matplotlib to create an error bandwidth"""

    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dynamic_scenarios = (20, 15, 10)
    Towns = (1,2)
    mode = "SLAM"

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
        plt.title("RMSE Relative Pose Error translational component")
        # total number of entries
        N = (len(dynamic_scenarios) + 1) * len(starting_locations)
        i = 0
        ind = np.arange(N)
        x_scatter = [0]*5*N
        for i in range(N):
            x_scatter[5*i:5*i+5] = [ind[i]]*5
        x = []
        for index, SL in enumerate(starting_locations):
            avg_trans = []
            avg_trans_upp = []
            avg_trans_low = []
            avg_rot = []
            avg_rot_upp = []
            avg_rot_low = []

            scatter_trans = []
            scatter_rot = []

            track_fail = []
            loop_fail = []

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
                    avg_trans_upp.extend([Perf.rmse_static_avg[0] + 2*Perf.rmse_static_std[0],
                                          Perf.rmse_dynamic_avg[0] + 2*Perf.rmse_dynamic_std[0]])
                    avg_trans_low.extend([Perf.rmse_static_avg[0] - 2 * Perf.rmse_static_std[0],
                                          Perf.rmse_dynamic_avg[0] - 2 * Perf.rmse_dynamic_std[0]])

                    raw_trans_static = [raw_rmse_static[0] for raw_rmse_static in Perf.rmse_static]
                    raw_trans_dynamic = [raw_rmse_dynamic[0] for raw_rmse_dynamic in Perf.rmse_dynamic]
                    scatter_trans.extend([raw_trans_static, raw_trans_dynamic])

                    avg_rot.extend([Perf.rmse_static_avg[1], Perf.rmse_dynamic_avg[1]])
                    avg_rot_upp.extend([Perf.rmse_static_avg[1] + 2*Perf.rmse_static_std[1],
                                        Perf.rmse_dynamic_avg[1] + 2*Perf.rmse_dynamic_std[1]])
                    avg_rot_low.extend([Perf.rmse_static_avg[1] - 2 * Perf.rmse_static_std[1],
                                        Perf.rmse_dynamic_avg[1] - 2 * Perf.rmse_dynamic_std[1]])
                    track_fail.extend([Perf.lost_track_static, Perf.lost_track_dynamic])
                    loop_fail.extend([Perf.false_loop_static, Perf.false_loop_dynamic])

                else:
                    x.append("T" + str(Perf.location["Town"]) + "Nr" + str(index + 1) + "\n" + "Dist:" + str(
                         Perf.scenario["Distance"]))
                    avg_trans.append(Perf.rmse_dynamic_avg[0])
                    avg_trans_upp.append(Perf.rmse_dynamic_avg[0] + 2*Perf.rmse_dynamic_std[0])
                    avg_trans_low.append(Perf.rmse_dynamic_avg[0] - 2 * Perf.rmse_dynamic_std[0])

                    raw_trans_dynamic = [raw_rmse_dynamic[0] for raw_rmse_dynamic in Perf.rmse_dynamic]
                    scatter_trans.append(raw_trans_dynamic)

                    avg_rot.append(Perf.rmse_dynamic_avg[1])
                    avg_rot_upp.append(Perf.rmse_dynamic_avg[1] + 2*Perf.rmse_dynamic_std[1])
                    avg_rot_low.append(Perf.rmse_dynamic_avg[1] - 2*Perf.rmse_dynamic_std[1])
                    track_fail.append(Perf.lost_track_dynamic)
                    loop_fail.append(Perf.false_loop_dynamic)


            plt.subplot(2, 1, 1)
            # plt.grid(True)
            label_location = "T" + str(Perf.location["Town"]) + "Nr" + str(index + 1)
            pdb.set_trace()
            plt.plot(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans), color='green')
            plt.fill_between(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_trans_low), np.array(avg_trans_upp), color='green', alpha=0.5)
            # for j in range(len(scatter_trans)):
            #     pdb.set_trace()
            #     plt.scatter(np.array(x_scatter[j*5:j*5+1]), np.array(scatter_trans[j]))
            plt.subplots_adjust(hspace=0.5)
            plt.legend()

            plt.subplot(2, 1, 2)
            label_location = "T" + str(Perf.location["Town"]) + "Nr" + str(index + 1)
            plt.plot(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot), color='green',
                         label=label_location)
            plt.fill_between(ind[i:i + (len(dynamic_scenarios) + 1)], np.array(avg_rot_low), np.array(avg_rot_upp), color='green', alpha=0.5)
            plt.legend()

            pdb.set_trace()


        # width = 0.35
        # plt.subplot(3, 1, 3)
        # plot_track = plt.bar(ind[i:i + (len(dynamic_scenarios) + 1)], track_fail, width, color="r")
        # plot_loop = plt.bar(ind[i:i + (len(dynamic_scenarios) + 1)], loop_fail, width, bottom=track_fail, color="b")
            i = i + (len(dynamic_scenarios) + 1)

        plt.subplot(2, 1, 1)
        plt.xticks(ind, x)
        if mode == "SLAM" and Town == 1:
            plt.ylim(top=0.060)
        plt.xlim(right=N)
        plt.title("Translational component root mean square error of the relative pose error")
        plt.ylabel("Trans. RMSE RPE [-]")

        plt.subplot(2, 1, 2)
        plt.xticks(ind, x)
        if mode == "SLAM" and Town == 1:
            plt.ylim(top=0.040)
        plt.xlim(right=N)
        plt.title("Rotational component root mean square error relative pose error")
        plt.ylabel("Rot. RMSE RPE [deg/m]")
        # plt.subplot(3, 1, 3)
        # plt.title("Ratio removed due to false loop closure or tracking failure")
        # plt.xticks(ind, x)
        # plt.ylim(top=1.1)
        # plt.xlim(right=N)
        # plt.ylabel("Ratio removed of total data [-]")
        # plt.legend((plot_track, plot_loop), ("tracking failure", "false loop closure"))
        plt.show()


if __name__ == "__main__":
    main()
