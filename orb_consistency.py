from CarlaSlamPerformance import CarlaSlamEvaluate
from matplotlib import pyplot as plt
from evaluate_pose import evaluate_trajectory, evaluate_pose_over_time, difference_pose
from evaluate_RPE_dist import evaluate_RPE_dist
from evaluate_RPE_time import evaluate_RPE_time
from scenario_labelling import *

orb_static_1f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_1.txt"
orb_static_2f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_2.txt"
orb_static_3f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_3.txt"
orb_static_4f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_4.txt"
orb_static_5f = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_orb_5.txt"
orb_1 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_1.txt"
orb_2 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_2.txt"
orb_3 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_3.txt"
orb_4 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_4.txt"
orb_5 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_5.txt"
file_gt_dyn = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_gt.txt"
file_gt_stat = "/home/sietse/official_experiment_data/SL_40_NV_0_SV_1_gt.txt"

with CarlaSlamEvaluate("gt", file_gt_dyn, 'k-') as gt_dyn:
    gt_dyn.process_data()

with CarlaSlamEvaluate("gt", file_gt_stat, 'k--') as gt_stat:
    gt_stat.process_data()

with CarlaSlamEvaluate("orb", orb_static_1f, 'r--') as orb_static_1:
    orb_static_1.process_data()

with CarlaSlamEvaluate("orb", orb_static_2f, 'b--') as orb_static_2:
    orb_static_2.process_data()

with CarlaSlamEvaluate("orb", orb_static_3f, 'c--') as orb_static_3:
    orb_static_3.process_data()

with CarlaSlamEvaluate("orb", orb_static_4f, 'm--') as orb_static_4:
    orb_static_4.process_data()

with CarlaSlamEvaluate("orb", orb_static_5f, 'g--') as orb_static_5:
    orb_static_5.process_data()


with CarlaSlamEvaluate("orb", orb_1, 'r-') as orb_1:
    orb_1.process_data()

with CarlaSlamEvaluate("orb", orb_2, 'b-') as orb_2:
    orb_2.process_data()

with CarlaSlamEvaluate("orb", orb_3, 'c-') as orb_3:
    orb_3.process_data()

with CarlaSlamEvaluate("orb", orb_4, 'm-') as orb_4:
    orb_4.process_data()

with CarlaSlamEvaluate("orb", orb_5, 'g-') as orb_5:
        orb_5.process_data()


methods = [gt_dyn, orb_static_1, orb_static_2, orb_static_3, orb_static_4, orb_static_5, orb_1, orb_2, orb_3, orb_4, orb_5]

# evaluate_trajectory(methods)

GT = [gt_stat, gt_stat, gt_stat, gt_stat, gt_stat, gt_dyn, gt_dyn, gt_dyn, gt_dyn, gt_dyn]
SLAM = [orb_static_1, orb_static_2, orb_static_3, orb_static_4, orb_static_5, orb_1, orb_2, orb_3, orb_4, orb_5]
time_step = 1
evaluate_RPE_time(GT, SLAM, time_step)

distance = 50
evaluate_RPE_dist(GT, SLAM, distance)
# flocation = "/home/sietse/official_experiment_data/"
# with ScenarioProcessor(flocation=flocation, SL=40, NV=40) as SP:
#     dynamic_agents = SP.process_dynamic_agents()
#     hero = SP.process_hero()
#     encountered_vehicles = SP.encountered_vehicles_filter(hero, dynamic_agents)
#     plot_data_da = []
#     for encountered_vehicle in encountered_vehicles:
#         plot_data_da.append([encountered_vehicle.id, encountered_vehicle.begin_time, encountered_vehicle.end_time])

all_figures = plt.get_figlabels()
plt.show()