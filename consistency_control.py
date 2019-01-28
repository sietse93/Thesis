from CarlaSlamPerformance import CarlaSlamEvaluate
from matplotlib import pyplot as plt
import math
import numpy


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


    plt.figure("Pose difference between static and dynamic")
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

    plt.figure("Translational difference between static and dynamic")
    plt.plot(dynamic.time, error_mag)
    plt.xlabel("time [s]")
    plt.ylabel("magnitude error [m]")
    # notequal_traj_index = numpy.nonzero(error_mag)
    # print(dynamic.time[notequal_traj_index[0][0]])

def main():
    method_gt = "gt"
    gt_file1 = "/home/sietse/SL_58_NV_40_SV_1_gt.txt"
    gt_file2 = "/home/sietse/SL_58_NV_0_SV_1_gt.txt"

    with CarlaSlamEvaluate(method_gt, gt_file1) as gt_dynamic:
        gt_dynamic.process_data()

    with CarlaSlamEvaluate(method_gt, gt_file2) as gt_static:
        gt_static.process_data()

    gt_consistency(gt_dynamic, gt_static)
    plt.show()



if __name__ == "__main__":
    main()
