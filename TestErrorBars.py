from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from matplotlib import pyplot as plt
import numpy as np
import pdb

def main():
    """Test if error bars show the results instinctively"""
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    Town = 1
    starting_locations = (0, 27, 58)
    dynamic_scenarios = (20, 15)

    # for SL in starting_locations:
    SL = 58
    x = []
    y_mean = []
    y_std = []
    all_perf = []
    for ds in dynamic_scenarios:
        dir_name_stat = DirName(Town, SL, "static")
        dir_name_dyn = DirName(Town, SL, "dynamic", ds)

        orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat)
        orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)
        Perf = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
        all_perf.append(Perf)

        print("converted")

        if ds == 20:
            x.extend(["static", str(ds)])
            y_mean.extend([Perf.rmse_static_avg[0], Perf.rmse_dynamic_avg[0]])
            y_std.extend([Perf.rmse_static_std[0], Perf.rmse_dynamic_std[0]])
        else:
            x.append(str(ds))
            y_mean.append(Perf.rmse_dynamic_avg[0])
            y_std.append(Perf.rmse_dynamic_std[0])

    plt.figure("error plot")
    plt.errorbar(np.array(x), np.array(y_mean), np.array(y_std), label=SL)
    plt.legend()
    plt.show()



if __name__ == "__main__":
    main()
