import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import pdb
from main_InspectData import InspectJsonFileInDir
from class_ScenarioLocationPerformance import *

# quick test to see if we can create a pandas struct fast and if we can use seaborn to create a errorband plot

# set the columns for the pandas data
columns = ["Scenario",
           "Dynamic variable",
           "Town",
           "Starting location",
           "trajectory",
           "SLAM type",
           "RMSE trans",
           "RMSE rot",
           "lost track",
           "false relocalization"]


Town = 1
dv = 20
SL = 0
base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
dir_stat = "T{}_SL{}_s/".format(Town, SL)
dir_dyn = "T{}_SL{}_d{}/".format(Town, SL, dv)
SLAM_mode = "SLAM"
Scenario = "Behind van"

# read the raw data files
orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_stat, SLAM_mode)
orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_dyn, SLAM_mode)

# get the rmse values
performance_class = ScenarioLocationPerformance(dv, Town, SL, orb_static, orb_dynamic, gt)

# put the data in pandas format
panda_data_static = []
panda_data_dynamic = []
for index in range(5):
    if Town == 1 and SL == 0:
        trajectory_dynamic = "T{}Nr{}\nDist: {}".format(Town, 1, dv)
        trajectory_static = "T{}Nr{}\nStatic".format(Town, 1)
    panda_data_static.append(["Static", np.nan, Town, SL, trajectory_static, SLAM_mode,
                              performance_class.rmse_static[index][0],
                              performance_class.rmse_static[index][1],
                             performance_class.lost_track_static,
                             performance_class.false_loop_static])
    panda_data_dynamic.append([Scenario, dv, Town, SL, trajectory_dynamic, SLAM_mode,
                              performance_class.rmse_dynamic[index][0],
                              performance_class.rmse_dynamic[index][1],
                             performance_class.lost_track_dynamic,
                             performance_class.false_loop_dynamic])
data = panda_data_static
data.extend(panda_data_dynamic)

# create the data in pandas struct - this is all data
df = pd.DataFrame(data, range(len(data)), columns)

# this is data from an tutorial
fmri = sns.load_dataset("fmri")
x_values = df["trajectory"]
y_values = df["RMSE trans"]
sns.relplot(x=x_values, y=y_values, kind="line")
plt.show()
