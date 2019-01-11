from CarlaSlamPerformance import CarlaSlamEvaluate, evaluate_trajectory, evaluate_pose_over_time
from matplotlib import pyplot as plt
import math


def gt_consistency(dynamic, static):
    error_x = []
    error_y = []
    error_z = []
    error_mag = []

    for i in range(0, len(dynamic.positions), 1):
        e_x = dynamic.positions[i][0] - static.positions[i][0]
        e_y = dynamic.positions[i][1] - static.positions[i][1]
        e_z = dynamic.positions[i][2] - static.positions[i][2]
        error_x.append(e_x)
        error_y.append(e_y)
        error_z.append(e_z)
        error_mag.append(math.sqrt(e_x**2 + e_y**2 + e_z**2))


    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(dynamic.time, error_x)
    plt.xlabel("time [s]")
    plt.ylabel("error in x [m]")

    plt.subplot(3, 1, 2)
    plt.plot(dynamic.time, error_y)
    plt.xlabel("time [s]")
    plt.ylabel("error in y [m]")

    plt.subplot(3, 1, 3)
    plt.plot(dynamic.time, error_z)
    plt.xlabel("time [s]")
    plt.ylabel("error in z [m]")

    plt.figure()
    plt.plot(dynamic.time, error_mag)
    plt.xlabel("time [s]")
    plt.ylabel("magnitude error [m]")


def main():
    method_gt = "gt"
    gt_file1 = "/home/sietse/CheckApConsistency/60fps/SL_12_NV_40_SV_1_gt.txt"
    # gt_file1 = "/home/sietse/carla_experiment_data/closed_loop/SL_144_NV_0_SV_1_gt.txt"
    gt_file2 = "/home/sietse/CheckApConsistency/60fps/SL_12_NV_0_SV_1_gt.txt"

    with CarlaSlamEvaluate(method_gt, gt_file1) as gt_dynamic:
        gt_dynamic.process_data()

    with CarlaSlamEvaluate(method_gt, gt_file2) as gt_static:
        gt_static.process_data()

    evaluate_trajectory([gt_dynamic, gt_static])
    evaluate_pose_over_time([gt_dynamic], [gt_static])
    gt_consistency(gt_dynamic, gt_static)
    plt.show()


if __name__ == "__main__":
    main()
