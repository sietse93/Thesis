import json
from func_Convert2Json import json2crf
from evaluate_pose import *
from class_ScenarioLocationPerformance import *
from matplotlib import pyplot as plt
import pdb

# check on 1 starting location with 1 scenario, look at the poses, select inliers

def main():
    Town = 3
    SL = 132
    ds = 10

    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    dir_name_stat = DirName(Town, SL, "static")
    dir_name_dyn = DirName(Town, SL, "dynamic", ds)

    # inspect the data
    orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat)
    orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)
    methods = []
    SLAM = []
    GT = []
    for orb in orb_static:
        methods.append(orb)
        SLAM.append(orb)
        GT.append(gt)
    for orb in orb_dynamic:
        methods.append(orb)
        SLAM.append(orb)
        GT.append(gt)
    # pdb.set_trace()
    methods.append(gt)

    VisualizeData(methods)

    LocalPerformance = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)

    LocalPerformance.ShowRpeDistAll()
    LocalPerformance.ShowRpeDistFiltered()
    LocalPerformance.SummaryPerformance()

    plt.show()

    # reject the outliers
    # selection = range(5)
    # dir = base_dir + dir_name
    # ExportOrbSelection(selection, dir)


def DirName(Town, SL, scenario, ds=20):
    """Gets the specific directory which contain the files of this specific scenario"""
    if scenario == "dynamic":
        dir_name = "T{}_SL{}_d{}/".format(Town, SL, ds)
    elif scenario == "static":
        dir_name = "T{}_SL{}_s/".format(Town, SL)
    else:
        print("scenario is not 'static' nor 'dynamic' ")
        return
    return dir_name


def InspectJsonFileInDir(Town, SL, base_dir, dir_name):
    """import all JSON files in directory and outputs a list of crf objects and a groundtruth"""
    dir = base_dir + dir_name
    file_ext = "_orb_{}_json.txt"

    # get the orb data for a scenario
    orb_data = []
    methods = []
    for i in range(5):
        file_name = dir_name[:-1]+file_ext.format(i)
        orb = json2crf(dir, file_name)
        orb_data.append(orb)
        methods.append(orb)

    dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
    dir_gt = base_dir + dir_name_stat
    file_name_gt = dir_name_stat[:-1]+"_gt_json.txt"
    gt = json2crf(dir_gt, file_name_gt)

    return orb_data, gt


def VisualizeData(methods):
    evaluate_trajectory(methods)
    compare_position(methods)
    compare_euler_angles(methods)


def ExportOrbSelection(selection, dir):
    file_name = dir + "orb_selection.txt"
    json_file = open(file_name, 'w')
    json_data = json.dumps(selection)
    json_file.write(json_data)
    json_file.close()


if __name__ == "__main__":
    main()


