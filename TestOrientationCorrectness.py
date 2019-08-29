from CarlaSlamPerformanceOrientationTest import CarlaSlamEvaluate
from evaluate_pose import *
from matplotlib import pyplot as plt
from evaluate_RPE_time import evaluate_RPE_time
from evaluate_RPE_dist import evaluate_RPE_dist

# file_gt_stat = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_gt.txt"
# file_string_static = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_orb_1.txt"
file_gt_stat = "/home/sietse/results_carla0.9/T3_SL97_s/T3_SL97_s_gt.txt"
file_string_static = "/home/sietse/results_carla0.9/T3_SL97_s/T3_SL97_s_orb_0.txt"

with CarlaSlamEvaluate("gt", file_gt_stat, 'k-') as gt:
    gt.process_data()

with CarlaSlamEvaluate("orb", file_string_static, 'b-') as orb:
    orb.process_data()

evaluate_trajectory([gt])
compare_position([gt])
evaluate_pose_over_time([gt], [orb])
difference_pose([gt], [orb])
# evaluate_RPE_time([gt], [orb], 1.0)
evaluate_RPE_dist([gt], [orb], 100)
plt.show()