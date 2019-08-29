from class_ConvertRefFrame import ConvertRefFrame
from evaluate_pose import *
from matplotlib import pyplot as plt

Town = 1
SL = 0
ds = 20
base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

scenario = "static"

file_name = "T{}_SL{}_{}_gt.txt".format(Town, SL, scenario[0])
dir_name = "T{}_SL{}_{}/".format(Town, SL, scenario[0])
file_loc = base_dir + dir_name + file_name

with ConvertRefFrame("gt", file_loc, "k-") as gt:
    gt.process_data()

methods = [gt]
evaluate_trajectory(methods)
compare_position(methods)
compare_euler_angles(methods)

plt.show()