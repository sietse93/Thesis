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

    plt.figure("lala")
    for SL in starting_locations:
        x = []
        y_mean = []
        y_std = []
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
                x.extend([str(Perf.location["Location"]) + "," + "static",
                          str(Perf.location["Location"]) + "," + str(Perf.scenario["Distance"])])
                y_mean.extend([Perf.rmse_static_avg[0], Perf.rmse_dynamic_avg[0]])
                y_std.extend([Perf.rmse_static_std[0], Perf.rmse_dynamic_std[0]])
                track_fail.extend([Perf.lost_track_static, Perf.lost_track_dynamic])
                loop_fail.extend([Perf.false_loop_static, Perf.false_loop_dynamic])

            else:
                x.append(str(Perf.location["Location"]) + "," + str(Perf.scenario["Distance"]))
                y_mean.append(Perf.rmse_dynamic_avg[0])
                y_std.append(Perf.rmse_dynamic_std[0])
                track_fail.append(Perf.lost_track_dynamic)
                loop_fail.append(Perf.false_loop_dynamic)

        plt.subplot(2, 1, 1)
        # plt.grid(True)
        plt.errorbar(np.array(x), np.array(y_mean), np.array(y_std), label=Perf.location)
        plt.legend()
        width = 0.35
        plt.subplot(2, 1, 2)
        plot_track = plt.bar(np.array(x), track_fail, width, color="r")
        plot_loop = plt.bar(np.array(x), loop_fail, width, bottom=plot_track, color="b")

    plt.legend((plot_track, plot_loop), ("tracking failure", "loop failure"))
    plt.show()


if __name__ == "__main__":
    main()
