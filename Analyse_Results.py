from CarlaSlamPerformance import *
from consistency_control import *


def main():
    method_gt = "gt"

    gt_file_static = "/home/sietse/SL_58_NV_0_SV_1_gt.txt"
    gt_file_dynamic = "/home/sietse/SL_58_NV_40_SV_1_gt.txt"

    with CarlaSlamEvaluate(method_gt, gt_file_static) as gt_static:
        gt_static.process_data()

    with CarlaSlamEvaluate(method_gt, gt_file_dynamic) as gt_dynamic:
        gt_dynamic.process_data()

    method_orb = "orb"
    orb_file_static = "/home/sietse/SL_58_NV_0_SV_1_orb.txt"
    orb_file_dynamic = "/home/sietse/SL_58_NV_40_SV_1_orb.txt"

    with CarlaSlamEvaluate(method_orb, orb_file_static) as orb_static:
        orb_static.process_data()

    with CarlaSlamEvaluate(method_orb, orb_file_dynamic) as orb_dynamic:
        orb_dynamic.process_data()

    time_step = 1

    gt_consistency(gt_dynamic, gt_static)
    evaluate_objects = [gt_static, orb_static, orb_dynamic]
    compare_position(evaluate_objects)
    compare_euler_angles(evaluate_objects)
    evaluate_trajectory(evaluate_objects)
    evaluate_RPE([gt_static, gt_dynamic], [orb_static, orb_dynamic], time_step=time_step)
    plt.show()


if __name__ == "__main__":
    main()