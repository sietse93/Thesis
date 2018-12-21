from CarlaSlamPerformance import CarlaSlamEvaluate, evaluate_trajectory


def main():
    method_gt = "gt"
    gt_file1 = "/home/sietse/carla_experiment_data/closed_loop/SL_144_NV_40_SV_1_gt.txt"
    gt_file2 = "/home/sietse/carla_experiment_data/closed_loop/SL_144_NV_0_SV_1_gt.txt"

    with CarlaSlamEvaluate(method_gt, gt_file1) as gt_dynamic:
        gt_dynamic.process_data()

    with CarlaSlamEvaluate(method_gt, gt_file2) as gt_static:
        gt_static.process_data()


if __name__ == "__main__":
    main()
