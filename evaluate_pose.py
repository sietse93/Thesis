from CarlaSlamPerformance import CarlaSlamEvaluate
from matplotlib import pyplot as plt

def compare_position(methods):
    """plots the position of a list of CarlaSlamEvaluate objects"""

    plt.figure("Pose over time")

    for method in methods:
        x = [position[0] for position in method.positions]
        y = [position[1] for position in method.positions]
        z = [position[2] for position in method.positions]

        plt.subplot(3, 1, 1)
        plt.plot(method.time, x, method.plotstyle, label=method.label)
        plt.xlabel("time [s]")
        plt.ylabel("x location [m]")

        plt.subplot(3, 1, 2)
        plt.plot(method.time, y, method.plotstyle, label=method.label)
        plt.xlabel("time [s]")
        plt.ylabel("y location [m]")

        plt.subplot(3, 1, 3)
        plt.plot(method.time, z, method.plotstyle, label=method.label)
        plt.xlabel("time [s]")
        plt.ylabel("z location [m]")
    plt.legend()


def difference_pose(GT, SLAM):
    """Plots the difference in position over time in the 3 axis"""

    plt.figure("Difference Pose over time")

    for gt, Slam in zip(GT, SLAM):
        diff_x = []
        diff_y = []
        diff_z = []
        diff_roll = []
        diff_pitch = []
        diff_yaw = []
        time_used = []
        for timestamp, position, orientation in zip(Slam.time, Slam.positions, Slam.orientations):
            try:
                eq_gt_index = gt.time.index(timestamp)
                time_used.append(gt.time[eq_gt_index])
                gt_position = gt.positions[eq_gt_index]
                gt_orientation = gt.orientations[eq_gt_index]
                diff_x.append(position[0] - gt_position[0])
                diff_y.append(position[1] - gt_position[1])
                diff_z.append(position[2] - gt_position[2])
                diff_roll.append(orientation[0]-gt_orientation[0])
                diff_pitch.append(orientation[1] - gt_orientation[1])
                diff_yaw.append(orientation[2]-gt_orientation[2])
            except ValueError:
                continue

        plt.subplot(3, 2, 1)
        plt.plot(time_used, diff_x, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel(" difference in x [m]")

        plt.subplot(3,2,3)
        plt.plot(time_used, diff_y, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel(" difference in y [m]")

        plt.subplot(3,2,5)
        plt.plot(time_used, diff_z, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel(" difference in z [m]")

        plt.subplot(3,2,2)
        plt.plot(time_used, diff_roll, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel(" difference in roll [deg]")

        plt.subplot(3,2,4)
        plt.plot(time_used, diff_pitch, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel(" difference in pitch [deg]")

        plt.subplot(3,2,6)
        plt.plot(time_used, diff_yaw, Slam.plotstyle, label=Slam.label)
        plt.xlabel("time [s]")
        plt.ylabel(" difference in yaw [deg]")

        plt.legend()


def compare_quaternions(methods):
    """Plots the quaternions of a list of CarlaSlamEvaluate objects """
    plt.figure("Quaternions")

    for method in methods:
        q1 = [quaternion[0] for quaternion in method.quaternions]
        q2 = [quaternion[1] for quaternion in method.quaternions]
        q3 = [quaternion[2] for quaternion in method.quaternions]
        q4 = [quaternion[3] for quaternion in method.quaternions]

        plt.subplot(4, 1, 1)
        plt.plot(method.time, q1, method.plotstyle, label=method.label)
        plt.subplot(4, 1, 2)
        plt.plot(method.time, q2, method.plotstyle, label=method.label)
        plt.subplot(4, 1, 3)
        plt.plot(method.time, q3, method.plotstyle, label=method.label)
        plt.subplot(4, 1, 4)
        plt.plot(method.time, q4, method.plotstyle, label=method.label)

    plt.legend()


def compare_euler_angles(methods):
    plt.figure("Euler_angles")

    plt.subplot(3, 1, 1)
    plt.xlabel("time [s]")
    plt.ylabel("roll [deg]")

    plt.subplot(3, 1, 2)
    plt.xlabel("time [s]")
    plt.ylabel("pitch [deg]")

    plt.subplot(3, 1, 3)
    plt.xlabel("time [s]")
    plt.ylabel("yaw [deg]")

    for method in methods:
        roll = [orientation[0] for orientation in method.orientations]
        pitch = [orientation[1] for orientation in method.orientations]
        yaw = [orientation[2] for orientation in method.orientations]

        plt.subplot(3, 1, 1)
        plt.plot(method.time, roll, method.plotstyle, label=method.label)

        plt.subplot(3, 1, 2)
        plt.plot(method.time, pitch, method.plotstyle, label=method.label)

        plt.subplot(3, 1, 3)
        plt.plot(method.time, yaw, method.plotstyle, label=method.label)

    plt.legend()

def evaluate_trajectory(methods):

    plt.figure("Trajectory")
    plt.title('Trajectory')
    plt.xlabel("x position")
    plt.ylabel("y position")

    for method in methods:
        x = [position[0] for position in method.positions]
        y = [position[1] for position in method.positions]
        plt.plot(x, y, method.plotstyle, label=method.label)
    plt.plot(methods[0].positions[0][0], methods[0].positions[0][1], 'x', label="starting point")

    plt.legend()


def evaluate_pose_over_time(GT, SLAM):

    """Plots the pose (xyz and Euler angles) over time of both the groundtruth and the SLAM algorithm"""

    # create layout for the plots
    # plt.figure("Pose over time")
    plt.figure("Pose over time")
    plt.title('Pose over time')

    plt.subplot(3, 2, 1)
    plt.xlabel("time [s]")
    plt.ylabel("x position")

    plt.subplot(3, 2, 3)
    plt.xlabel("time [s]")
    plt.ylabel("y position")

    plt.subplot(3, 2, 5)
    plt.xlabel("time [s]")
    plt.ylabel("z position")

    plt.subplot(3, 2, 2)
    plt.xlabel("time [s]")
    plt.ylabel("roll [deg]")

    plt.subplot(3, 2, 4)
    plt.xlabel("time [s]")
    plt.ylabel("pitch [deg]")

    plt.subplot(3, 2, 6)
    plt.xlabel("time [s]")
    plt.ylabel("yaw [deg]")

    # plot all the groundtruths
    for gt in GT:
        gt_x = [position[0] for position in gt.positions]
        gt_y = [position[1] for position in gt.positions]
        gt_z = [position[2] for position in gt.positions]
        gt_roll = [orientation[0] for orientation in gt.orientations]
        gt_pitch = [orientation[1] for orientation in gt.orientations]
        gt_yaw = [orientation[2] for orientation in gt.orientations]

        plt.subplot(3, 2, 1)
        plt.plot(gt.time, gt_x, gt.plotstyle, label=gt.label)

        plt.subplot(3, 2, 3)
        plt.plot(gt.time, gt_y, gt.plotstyle, label=gt.label)

        plt.subplot(3, 2, 5)
        plt.plot(gt.time, gt_z, gt.plotstyle, label=gt.label)

        plt.subplot(3, 2, 2)
        plt.plot(gt.time, gt_roll, gt.plotstyle, label=gt.label)

        plt.subplot(3, 2, 4)
        plt.plot(gt.time, gt_pitch, gt.plotstyle, label=gt.label)

        plt.subplot(3, 2, 6)
        plt.plot(gt.time, gt_yaw, gt.plotstyle, label=gt.label)

    # plot all the SLAM data
    for Slam in SLAM:
        Slam_x = [position[0] for position in Slam.positions]
        Slam_y = [position[1] for position in Slam.positions]
        Slam_z = [position[2] for position in Slam.positions]
        Slam_roll = [orientation[0] for orientation in Slam.orientations]
        Slam_pitch = [orientation[1] for orientation in Slam.orientations]
        Slam_yaw = [orientation[2] for orientation in Slam.orientations]

        plt.subplot(3, 2, 1)
        plt.plot(Slam.time, Slam_x, Slam.plotstyle, label=Slam.label)

        plt.subplot(3, 2, 3)
        plt.plot(Slam.time, Slam_y, Slam.plotstyle, label=Slam.label)

        plt.subplot(3, 2, 5)
        plt.plot(Slam.time, Slam_z, Slam.plotstyle, label=Slam.label)

        plt.subplot(3, 2, 2)
        plt.plot(Slam.time, Slam_roll, Slam.plotstyle, label=Slam.label)

        plt.subplot(3, 2, 4)
        plt.plot(Slam.time, Slam_pitch, Slam.plotstyle, label=Slam.label)

        plt.subplot(3, 2, 6)
        plt.plot(Slam.time, Slam_yaw, Slam.plotstyle, label=Slam.label)

    plt.legend()

def main():
    method_gt = "gt"
    gt_ps_static = 'k-'
    gt_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_gt.txt"
    with CarlaSlamEvaluate(method_gt, gt_file_static, gt_ps_static) as gt_static:
        gt_static.process_data()

    method_orb = "orb"
    orb_ps_static = 'g-'
    orb_file_static = "/home/sietse/PrelimExpStaticVsDynamic/SL_58_NV_0_SV_1_orb.txt"

    with CarlaSlamEvaluate(method_orb, orb_file_static, orb_ps_static) as orb_static:
        orb_static.process_data()

    plt.rcParams['axes.grid'] = True

    # methods does not compute anything, it just plots
    methods = [gt_static, orb_static]
    GT = [gt_static]
    SLAM = [orb_static]
    # compare_position(methods)
    # difference_positions(GT, SLAM)
    # compare_quaternions(methods)
    # compare_euler_angles(methods)
    # evaluate_trajectory(methods)
    # evaluate_pose_over_time(GT, SLAM)

    plt.show()

if __name__ == "__main__":
    main()