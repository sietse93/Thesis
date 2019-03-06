from consistency_control import gt_consistency
from evaluate_RPE_dist import evaluate_RPE_dist
from evaluate_RPE_time import evaluate_RPE_time
from evaluate_pose import *
from matplotlib import pyplot as plt
from CarlaSlamPerformance import CarlaSlamEvaluate
from average_orb import *
from evaluate_RPE_time_orbAvgDebug import evaluate_RPE_time_debug
from scenario_labelling import ScenarioProcessor

# file locations
file_string_static = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_{}.txt"
file_string_dynamic = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_{}.txt"
file_gt_dyn = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_gt.txt"
file_gt_stat = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_gt.txt"

# list with all orb data in CarlaSlamEvaluate objects
orb_static_objects = []
orb_dynamic_objects = []

# read all orb data
for i in range(5):
    i = i+1
    with CarlaSlamEvaluate("orb", file_string_static.format(i), 'b-') as orb_static:
        orb_static.process_data()
    orb_static_objects.append(orb_static)

    with CarlaSlamEvaluate("orb", file_string_dynamic.format(i), 'r--') as orb_dynamic:
        orb_dynamic.process_data()
    orb_dynamic_objects.append(orb_dynamic)

# read ground truth data
with CarlaSlamEvaluate("gt", file_gt_stat, 'k-') as gt_static:
    gt_static.process_data()

with CarlaSlamEvaluate("gt", file_gt_dyn, 'k--') as gt_dyn:
    gt_dyn.process_data()

print("data converted into objects")
# average the data
time_step = 0.025
AverageStatic = average_orb(orb_static_objects, time_step, 'orb_static', 'b-')
AverageDynamic = average_orb(orb_dynamic_objects, time_step, 'orb_dynamic', 'r--')

print("data averaged")

flocation = "/home/sietse/official_experiment_data/"
with ScenarioProcessor(flocation=flocation, SL=40, NV=40) as SP:
    dynamic_agents = SP.process_dynamic_agents()
    hero = SP.process_hero()
    encountered_vehicles = SP.encountered_vehicles_filter(hero, dynamic_agents)
    SP.vehicle_labeling(encountered_vehicles)

print("encountered vehicles processed")


# compare their poses over time
# methods = [AverageStatic, AverageDynamic]
# compare_position(methods)
# compare_euler_angles(methods)
# compare_quaternions(methods)
# evaluate RPE over time
GT = [gt_static, gt_dyn]
SLAM = [AverageStatic, AverageDynamic]
time_RPE = 1.0
evaluate_RPE_time(GT, SLAM, time_RPE, encountered_vehicles)

plt.show()