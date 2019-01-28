from CarlaSlamPerformance import CarlaSlamEvaluate
from consistency_control import gt_consistency
from evaluate_RPE_dist import evaluate_RPE_dist
from evaluate_RPE_time import evaluate_RPE_time
from evaluate_pose import evaluate_pose_over_time, evaluate_trajectory
from matplotlib import pyplot as plt
from load_experiment import load_experiment


def main():
    flocation = "/home/sietse/official_experiment_data/"
    SL = 105
    (sl144_gt_static, sl144_gt_dynamic, sl144_orb_static, sl144_orb_dynamic) = load_experiment(flocation, SL)

    # method_gt = "gt"
    #
    # floc_sl144_gt_static = "/home/sietse/official_experiment_data/SL_144_NV_0_SV_1_gt.txt"
    # floc_sl144_gt_dynamic = "/home/sietse/official_experiment_data/SL_144_NV_0_SV_1_gt.txt"
    #
    # with CarlaSlamEvaluate(method_gt, floc_sl144_gt_static, 'k-') as sl144_gt_static:
    #     sl144_gt_static.process_data()
    #
    # with CarlaSlamEvaluate(method_gt, floc_sl144_gt_dynamic, 'k--') as sl144_gt_dynamic:
    #     sl144_gt_dynamic.process_data()
    #
    # method_orb = "orb"
    # floc_sl144_orb_static = "/home/sietse/official_experiment_data/SL_144_NV_0_SV_1_orb.txt"
    # floc_sl144_orb_dynamic = "/home/sietse/official_experiment_data/SL_144_NV_40_SV_1_orb.txt"
    #
    # with CarlaSlamEvaluate(method_orb, floc_sl144_orb_static, 'r-') as sl144_orb_static:
    #     sl144_orb_static.process_data()
    #
    # with CarlaSlamEvaluate(method_orb, floc_sl144_orb_dynamic, 'b--') as sl144_orb_dynamic:
    #     sl144_orb_dynamic.process_data()
    #

    # First step is to check ground truth consistency between dynamic and ground truth trajectory
    gt_consistency(sl144_gt_dynamic, sl144_gt_static)

    # If the error is negligible only one gt suffices in the method list
    methods = [sl144_gt_static, sl144_orb_static, sl144_orb_dynamic]

    # Check the overall estimated trajectory
    evaluate_trajectory(methods)

    # Check the pose error over time
    # GT = [sl20_gt_static, sl20_gt_dynamic]
    # SLAM = [sl20_orb_static, sl20_orb_dynamic]
    # evaluate_pose_over_time(GT, SLAM)
    #
    # distance = 50
    # evaluate_RPE_dist(GT, SLAM, distance)
    plt.show()
    # time_step = 2


    # Evaluate actual pose over time
    # compare_position(evaluate_objects)
    # compare_euler_angles(evaluate_objects)
    # evaluate_trajectory(evaluate_objects)

    # Evaluate Relative Pose Error
    # evaluate_RPE([gt_static], [orb_static], time_step=time_step)
    # evaluate_RPE([gt_static, gt_dynamic], [orb_static, orb_dynamic], time_step=time_step)

    # Difference between the dynamic groundtruth and static groundtruth
    # gt_consistency(gt_dynamic, gt_static)

    # difference_positions([gt_static, gt_dynamic], [orb_static, orb_dynamic])

    # evaluate_RPE_dist([gt_static, gt_dynamic], [orb_static, orb_dynamic], distance)
    # plt.show()


if __name__ == "__main__":
    main()
