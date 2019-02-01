from CarlaSlamPerformance import CarlaSlamEvaluate
from consistency_control import gt_consistency
from evaluate_RPE_dist import evaluate_RPE_dist
from evaluate_RPE_time import evaluate_RPE_time
from evaluate_pose import evaluate_pose_over_time, evaluate_trajectory
from matplotlib import pyplot as plt
from load_experiment import load_experiment


def main():
    flocation = "/home/sietse/official_experiment_data/"
    SL = 144
    (gt_static, gt_dynamic, orb_static, orb_dynamic) = load_experiment(flocation, SL)


    # First step is to check ground truth consistency between dynamic and ground truth trajectory
    # gt_consistency(gt_dynamic, gt_static)

    # If the error is negligible only one gt suffices in the method list
    methods = [gt_static, orb_static, orb_dynamic]

    # Check the overall estimated trajectory
    evaluate_trajectory(methods)

    # Check the pose error over time
    GT = [gt_static, gt_dynamic]
    SLAM = [orb_static, orb_dynamic]
    evaluate_pose_over_time(GT, SLAM)

    distance = 50
    evaluate_RPE_dist(GT, SLAM, distance)
    # print("mean RPE for orb static is {} \n mean RPE for orb dynamic is {}".format(orb_static.RPE_dist_mean, orb_dynamic.RPE_dist_mean))
    time_step = 1
    evaluate_RPE_time(GT, SLAM, time_step)
    plt.show()



if __name__ == "__main__":
    main()
