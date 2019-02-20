from CarlaSlamPerformance import CarlaSlamEvaluate
from matplotlib import pyplot as plt
from evaluate_pose import evaluate_trajectory

# file_orb_full_rate = "/home/sietse/official_experiment_data/orb_consistency/SL20_S_R1_V1.txt"
# file_orb_half_rate = "/home/sietse/official_experiment_data/orb_consistency/SL20_S_R05_V1.txt"
# file_gt = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_gt.txt"
#
# with CarlaSlamEvaluate("gt", file_gt, 'k-') as gt:
#     gt.process_data()
#
# with CarlaSlamEvaluate("orb", file_orb_full_rate, 'r-') as orb_full:
#     orb_full.process_data()
#
# with CarlaSlamEvaluate("orb", file_orb_half_rate, 'b-') as orb_half:
#     orb_half.process_data()
#
# methods = [gt, orb_full, orb_half]
#
# evaluate_trajectory(methods)
#
# plt.show()

file_orb_half_V1= "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb.txt"
file_orb_half_V2 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_V2.txt"
file_orb_half_V3 = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_orb_V3.txt"
file_gt = "/home/sietse/official_experiment_data/SL_40_NV_40_SV_1_gt.txt"

with CarlaSlamEvaluate("gt", file_gt, 'k-') as gt:
    gt.process_data()

with CarlaSlamEvaluate("orb", file_orb_half_V1, 'r-') as orb_V1:
    orb_V1.process_data()

with CarlaSlamEvaluate("orb", file_orb_half_V2, 'b-') as orb_V2:
    orb_V2.process_data()

with CarlaSlamEvaluate("orb", file_orb_half_V3, 'g-') as orb_V3:
    orb_V3.process_data()
methods = [gt, orb_V1, orb_V2, orb_V3]

evaluate_trajectory(methods)

plt.show()