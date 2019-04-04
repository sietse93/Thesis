from consistency_control import gt_consistency
from evaluate_RPE_dist import evaluate_RPE_dist
from evaluate_RPE_time import evaluate_RPE_time
from evaluate_RPE_time_da import evaluate_RPE_time_da
from evaluate_pose import *
from matplotlib import pyplot as plt
from CarlaSlamPerformance import CarlaSlamEvaluate
from average_orb import *
from scenario_labelling import ScenarioProcessor

## convert all orb data and gt data

# file locations
file_string_static = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_orb_{}.txt"
file_string_dynamic = "/home/sietse/official_experiment_data/SL_20_NV_40_SV_1_orb_{}.txt"
file_gt_dyn = "/home/sietse/official_experiment_data/SL_20_NV_40_SV_1_gt.txt"
file_gt_stat = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_gt.txt"

# list with all orb data in CarlaSlamEvaluate objects
orb_static_objects = []
orb_dynamic_objects = []
orb_all = []
#  read all orb data
for i in range(5):
    i = i+1
    static_plot = "C{}-".format(i)
    dynamic_plot = "C{}--".format(i)
    with CarlaSlamEvaluate("orb", file_string_static.format(i), static_plot) as orb_static:
        orb_static.process_data()
    orb_static_objects.append(orb_static)
    orb_all.append(orb_static)
    with CarlaSlamEvaluate("orb", file_string_dynamic.format(i), dynamic_plot) as orb_dynamic:
        orb_dynamic.process_data()
    orb_dynamic_objects.append(orb_dynamic)
    orb_all.append(orb_dynamic)

# read ground truth data
with CarlaSlamEvaluate("gt", file_gt_stat, 'k-') as gt_static:
    gt_static.process_data()

with CarlaSlamEvaluate("gt", file_gt_dyn, 'k--') as gt_dyn:
    gt_dyn.process_data()

gt_list = []
# for i in range(5):
#     gt_list.append(gt_static)
#     gt_list.append(gt_dyn)

plt.rcParams['axes.grid'] = True
# Analyze the data and see if some orb data influences the performance too much. Remove these orb data
#for i in range(1):
# orb_static_objects.pop(0)
orb_dynamic_objects.pop(0)
# orb_static_objects.pop(0)
# orb_dynamic_objects.pop(0)
l_orb = len(orb_dynamic_objects)
orb_all = []
for obj in orb_static_objects:
    orb_all.append(obj)
    gt_list.append(gt_static)
for obj in orb_dynamic_objects:
    orb_all.append(obj)
    gt_list.append(gt_dyn)


evaluate_RPE_time(gt_list, orb_all, 1.0)

print("data converted into objects")
# average the data
time_step = 0.025
AverageStatic = average_orb(orb_static_objects, time_step, 'orb_static', 'b-')
AverageDynamic = average_orb(orb_dynamic_objects, time_step, 'orb_dynamic', 'r--')

print("data averaged")

flocation = "/home/sietse/official_experiment_data/"
with ScenarioProcessor(flocation=flocation, SL=20, NV=40) as SP:
    dynamic_agents = SP.process_dynamic_agents()
    hero = SP.process_hero()
    encountered_vehicles = SP.encountered_vehicles_filter(hero, dynamic_agents)
    SP.vehicle_labeling(encountered_vehicles)

print("encountered vehicles processed")


# compare their poses over time
methods = [AverageStatic, AverageDynamic]
# compare_position(methods)
# compare_euler_angles(methods)
# compare_quaternions(methods)
# evaluate RPE over time
GT = [gt_static, gt_dyn]
SLAM = [AverageStatic, AverageDynamic]
time_RPE = 1.0
evaluate_RPE_time_da(GT, SLAM, time_RPE, encountered_vehicles)
# evaluate_RPE_time(GT, SLAM, time_RPE)
evaluate_RPE_dist(GT, SLAM, eva_dist=100)
#
plt.show()