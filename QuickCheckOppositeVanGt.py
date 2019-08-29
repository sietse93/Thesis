import json
from func_Convert2Json import json2crf
from evaluate_pose import *
from class_ScenarioLocationPerformance import *
from matplotlib import pyplot as plt
import pdb

Town = 1
SL = 0
nr_vans = 10

base_dir = "/home/sietse/results_carla0.9/VansOppositeRoad/"
dir_name = "T{}_SL{}_d{}/".format(Town, SL, nr_vans)
dir = base_dir+dir_name
file_ext = "_orb_{}_json.txt"

orb_data = []
for i in range(5):
    file_name = dir_name[:-1] + file_ext.format(i)
    orb = json2crf(dir, file_name)
    orb_data.append(orb)

file_name_gt = dir_name[:-1] + "_gt_json.txt"
gt_opposite = json2crf(dir, file_name_gt)

base_dir_stuck = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
dir_name_stuck = "T{}_SL{}_s/".format(Town, SL)
dir_stuck = base_dir_stuck + dir_name_stuck
file_name_gt_stuck = dir_name_stuck[:-1] + "_gt_json.txt"
gt_stuck = json2crf(dir_stuck, file_name_gt_stuck)


evaluate_trajectory([gt_opposite, gt_stuck])

diff = np.array([0.0, 0.0, 0.0])
for opposite, stuck in zip(gt_opposite.positions, gt_stuck.positions):
    diff += (opposite-stuck)

print(diff)

