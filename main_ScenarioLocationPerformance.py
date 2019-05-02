from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
import pdb

def main():
    """Describes the performance of a certain scenario in a certain starting location"""
    Town = 3
    SL = 127
    ds = 20
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    dir_name_stat = DirName(Town, SL, "static")
    dir_name_dyn = DirName(Town, SL, "dynamic", ds)
    orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat)
    orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)

    Test = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
    Test.SummaryPerformance()
    pdb.set_trace()
    # Test.ShowRpeDistFiltered()
    # plt.show()


if __name__ == "__main__":
    main()