import json
from func_ConvertCrf2Json import json2crf
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
from matplotlib import pyplot as plt
from main_InspectData import DirName, InspectJsonFileInDir
import pdb
import math


class ScSlPerf:
    def __init__(self, ds, Town, SL, slam_stat, slam_dyn, gt):
        self.scenario = {"Scenario": "Stuck behind van", " Distance": ds}
        self.location = {"Town": Town, "Location": SL}
        # contains the list of crs objects
        self.slam_stat = []
        self.slam_dyn = []
        self.raw_data_stat = []
        self.raw_data_dyn = []
        self.rmse_trans_static = []
        self.rmse_rot_static = []
        self.rmse_trans_dynamic = []
        self.rmse_rot_dynamic = []

        for orb in slam_stat:
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb)
            raw_data_single = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors,
                               "plotstyle": orb.plotstyle, "label": orb.label}
            self.slam_stat.append(slam_stat)
            self.raw_data_stat.append(raw_data_single)
            self.rmse_trans_static.append(calc_rmse(trans_errors))
            self.rmse_rot_static.append(calc_rmse(rot_errors))

        self.rmse_trans_static_avg = sum(self.rmse_trans_static)/len(self.rmse_trans_static)
        self.rmse_rot_static_avg = sum(self.rmse_rot_static)/len(self.rmse_rot_static)

        for orb in slam_dyn:
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb)
            raw_data_single = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors,
                               "plotstyle": orb.plotstyle, "label": orb.label}
            self.slam_dyn.append(slam_dyn)
            self.raw_data_dyn.append(raw_data_single)
            self.rmse_trans_dynamic.append(calc_rmse(trans_errors))
            self.rmse_rot_dynamic.append(calc_rmse(rot_errors))

        self.rmse_trans_dynamic_avg = sum(self.rmse_trans_dynamic)/len(self.rmse_trans_dynamic)
        self.rmse_rot_dynamic_avg = sum(self.rmse_rot_dynamic)/len(self.rmse_rot_dynamic)

        statvsdyn_trans = (self.rmse_trans_static_avg-self.rmse_trans_dynamic_avg)/self.rmse_trans_static_avg
        statvsdyn_rot = (self.rmse_rot_static_avg - self.rmse_rot_dynamic_avg) / self.rmse_rot_static_avg

        self.StatVsDyn = (statvsdyn_trans, statvsdyn_rot)

    def ShowScSl(self):
        for index, static_data in enumerate(self.raw_data_stat):
            t = static_data["time"]
            stat_trans = static_data["RPE_trans"]
            stat_rot = static_data["RPE_rot"]
            orb_label = static_data["label"]
            orb_plotstyle = static_data["plotstyle"]
            plt.figure("RPE Magnitude over distance")
            plt.subplot(2, 1, 1)
            plt.plot(t, stat_trans, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("translational error [-]")

            plt.subplot(2, 1, 2)
            plt.plot(t, stat_rot, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("rotational error [deg/m]")
            plt.legend()


        plt.show()





def main():
    """Describes the performance of a certain scenario in a certain starting location"""
    Town = 3
    SL = 132
    ds = 20
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    dir_name_stat = DirName(Town, SL, "static")
    dir_name_dyn = DirName(Town, SL, "dynamic", ds)
    orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat)
    orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)

    ScSlTest = ScSlPerf(ds, Town, SL, orb_static, orb_dynamic, gt)
    print(ScSlTest.rmse_trans_static_avg)
    print(ScSlTest.rmse_rot_static_avg)
    ScSlTest.ShowScSl()

    # raw_data = []
    # RMSE_trans_data = []
    # RMSE_rot_data = []
    # for orb in orb_static:
    #     time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb)
    #     raw_data_single = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors}
    #     raw_data.append(raw_data_single)
    #
    #     rmse_trans = calc_rmse(trans_errors)
    #     RMSE_trans_data.append(rmse_trans)
    #
    #     rmse_rot = calc_rmse(rot_errors)
    #     RMSE_rot_data.append(rmse_rot)
    #
    #     pdb.set_trace()



    # orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)

    # # get the directory
    # if scenario == "dynamic":
    #     dir_name = "T{}_SL{}_d{}/".format(Town, SL, ds)
    # elif scenario == "static":
    #     dir_name = "T{}_SL{}_s/".format(Town, SL)
    # else:
    #     print("scenario is not 'static' nor 'dynamic' ")
    #     return
    #
    # # get the inliers in orb slam selection
    # orb_selection_loc = base_dir+dir_name+"orb_selection.txt"
    # json_file = open(orb_selection_loc, 'r')
    # orb_indices = json.load(json_file)
    #
    # file_ext = "_orb_{}_json.txt"
    # file_loc = base_dir + dir_name
    # file_name = dir_name[:-1]
    # orb_data = []
    # for index in orb_indices:
    #     json_file_name = file_name + file_ext.format(index)
    #     orb = json2crf(file_loc, json_file_name)
    #     orb_data.append(orb)
    # print(len(orb_data))
    # dir_name_gt = "T{}_SL{}_s/".format(Town, SL)
    # json_name_gt = "T{}_SL{}_s_gt_json.txt".format(Town, SL)
    # gt = json2crf(base_dir + dir_name_gt, json_name_gt)
    #
    # gt_data = [gt for orb in orb_data]
    #
    # evaluate_RPE_dist(gt_data, orb_data, 100)
    # plt.show()


if __name__ == "__main__":
    main()