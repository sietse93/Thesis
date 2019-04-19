import json
from func_ConvertCrf2Json import json2crf
from evaluate_pose import *
from matplotlib import pyplot as plt

# check on 1 starting location with 1 scenario, look at the poses, select inliers

def main():
    Town = 1
    SL = 0
    ds = 15
    scenario = "dynamic"

    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    if scenario == "dynamic":
        dir_name = "T{}_SL{}_d{}/".format(Town, SL, ds)
    elif scenario == "static":
        dir_name = "T{}_SL{}_s/".format(Town, SL)
    else:
        print("scenario is not 'static' nor 'dynamic' ")
        return
    # inspect the data
    InspectJsonFile(Town, SL, base_dir, dir_name)

    # reject the outliers
    selection = range(5)
    dir = base_dir + dir_name
    ExportOrbSelection(selection, dir)

def InspectJsonFile(Town, SL, base_dir, dir_name):
    dir = base_dir + dir_name
    file_ext = "_orb_{}_json.txt"

    # get the orb data for a scenario
    orb_data = []
    for i in range(5):
        file_name = dir_name[:-1]+file_ext.format(i)
        orb = json2crf(dir, file_name)
        orb_data.append(orb)

    dir_name_stat = "T{}_SL{}_s/".format(Town, SL)
    dir_gt = base_dir + dir_name_stat
    file_name_gt = dir_name_stat[:-1]+"_gt_json.txt"
    gt = json2crf(dir_gt, file_name_gt)

    methods = orb_data
    methods.append(gt)
    evaluate_trajectory(methods)
    compare_position(methods)
    compare_euler_angles(methods)

    # plt.show()

def ExportOrbSelection(selection, dir):
    file_name = dir + "orb_selection.txt"
    json_file = open(file_name, 'w')
    json_data = json.dumps(selection)
    json_file.write(json_data)
    json_file.close()


if __name__ == "__main__":
    main()


