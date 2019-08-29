from class_ConvertRefFrame import ConvertRefFrame
from evaluate_pose import *
from main_InspectData import VisualizeData
from matplotlib import pyplot as plt

base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
DS = 20
Town = 1
SL = 0
scenario = "static"

dir_name = "T{}_SL{}_{}/".format(Town, SL, scenario[0])
file_dir = base_dir + dir_name

methods =[]
for i in range(2):
    file_name = "T{}_SL{}_{}_orb_{}.txt".format(Town, SL, scenario[0], i)

    file_loc = base_dir + dir_name + file_name

    with ConvertRefFrame("orb", file_loc, "C{}-".format(i)) as stat_orb:
        stat_orb.process_data()

    methods.append(stat_orb)


file_name = "T{}_SL{}_{}_gt.txt".format(Town, SL, scenario[0])
file_loc = base_dir + dir_name + file_name

with ConvertRefFrame("gt", file_loc, "k-") as gt:
    gt.process_data()
methods.append(gt)

compare_euler_angles(methods)

plt.show()

