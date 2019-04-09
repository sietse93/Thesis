from CarlaSlamPerformanceV2 import CarlaSlamEvaluate
from consistency_control import gt_consistency
from matplotlib import pyplot as plt
from evaluate_pose import evaluate_trajectory, compare_position
import pdb
gt_file_static = "/home/sietse/results_carla0.9/T3_SL97_s/T3_SL97_s_gt.txt"
gt_file_dynamic = "/home/sietse/results_carla0.9/T3_SL97_d_1/T3_SL97_d_1_gt.txt"

with CarlaSlamEvaluate("gt", gt_file_static, 'k-') as gt_static:
    gt_static.process_data()

pdb.set_trace()

with CarlaSlamEvaluate("gt", gt_file_dynamic, 'k--') as gt_dynamic:
    gt_dynamic.process_data()

# gt_consistency(gt_dynamic, gt_static)

evaluate_trajectory([gt_static])
compare_position([gt_static])

plt.show()
