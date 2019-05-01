import json
from func_ConvertCrf2Json import json2crf
from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
from matplotlib import pyplot as plt
from main_InspectData import DirName, InspectJsonFileInDir
import pdb
import math


class ScenarioLocationPerformance:
    def __init__(self, ds, Town, SL, slam_stat, slam_dyn, gt):
        # description of scenario
        self.scenario = {"Scenario": "Stuck behind van", " Distance": ds}
        self.location = {"Town": Town, "Location": SL}
        # contains the list of all crs objects static
        self.slam_stat = []
        # contains list of all crs objects dynamic
        self.slam_dyn = []

        # raw RPE dist data in a dict form. Everything is here to make a plot that explains everything
        self.raw_rpe_stat = []
        self.raw_rpe_dyn = []

        # Root mean square error translational and rotational dynamic and static of each sequence
        self.rmse_static = []
        self.rmse_dynamic = []

        # ratio that lost track of location
        self.lost_track_static = ()
        self.lost_track_dynamic = ()

        # average root mean square error translational and rotational component, static and dynamic
        self.rmse_static_avg = ()
        self.rmse_dynamic_avg = ()

        # Percentage improvement dynamic over static
        self.StatVsDynamic_avg = ()

        # save the crf data in lists
        for orb in slam_stat:
            self.slam_stat.append(orb)

        for orb in slam_dyn:
            self.slam_dyn.append(orb)

        # get the RPE performance from this data
        self.SaveRpeData(gt)
        filter_index_stat, filter_index_dyn = self.IndexUnsuccesfullTracking(gt)

        # find and filter all data that loose track during orb estimation

        # self.rmse_trans_dynamic_avg = sum(self.rmse_trans_dynamic)/len(self.rmse_trans_dynamic)
        # self.rmse_rot_dynamic_avg = sum(self.rmse_rot_dynamic)/len(self.rmse_rot_dynamic)
        #
        # statvsdyn_trans = (self.rmse_trans_static_avg-self.rmse_trans_dynamic_avg)/self.rmse_trans_static_avg
        # statvsdyn_rot = (self.rmse_rot_static_avg - self.rmse_rot_dynamic_avg) / self.rmse_rot_static_avg
        #
        # # Compare static vs dynamic
        # self.StatVsDyn = (statvsdyn_trans, statvsdyn_rot)

    def FilterSuccesfullTracking(self, gt):
        """Outputs all the indices that have not succesfully tracked the complete trajectory"""

        end_time = gt.time[-1]
        index_succes_tracking_stat = []
        nr_failed_tracking_stat = 0.0
        index_succes_tracking_dyn = []
        nr_failed_tracking_dyn = 0.0

        # time threshold where a trajectory is still completed wrt ground truth
        time_thres = 0.3

        for index, orb in enumerate(self.slam_stat):
            if (end_time - time_thres) < orb.time[-1] < end_time:
                index_succes_tracking_stat.append(index)
            else:
                nr_failed_tracking_stat += 1.0

        self.lost_track_static = nr_failed_tracking_stat/len(self.slam_stat)

        for index, orb in enumerate(self.slam_dyn):
            if (end_time - time_thres) < orb.time[-1] < end_time:
                index_succes_tracking_dyn.append(index)
            else:
                nr_failed_tracking_dyn += 1.0

        self.lost_track_dynamic = nr_failed_tracking_dyn/len(self.slam_dyn)

        return index_succes_tracking_stat, index_succes_tracking_dyn

    def SaveRpeData(self, gt):
        """Calculate the RPE dist for each sequence and the RMSE"""
        for orb in self.slam_stat:
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb, 100)
            RawRpeSingle = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors,
                               "plotstyle": orb.plotstyle, "label": orb.label}
            self.raw_rpe_stat.append(RawRpeSingle)
            orb_rmse = (calc_rmse(trans_errors), calc_rmse(rot_errors))
            self.rmse_static.append(orb_rmse)

        for orb in self.slam_dyn:
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb, 100)
            RawRpeSingle = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors,
                               "plotstyle": orb.plotstyle, "label": orb.label}
            self.raw_rpe_dyn.append(RawRpeSingle)
            orb_rmse = (calc_rmse(trans_errors), calc_rmse(rot_errors))
            self.rmse_dynamic.append(orb_rmse)



    def ShowRpeDistAll(self):
        """Plots the RPE dist of all the data that is used to make a Scenario Location specific performance class"""
        for index, static_data in enumerate(self.raw_rpe_stat):
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

        for index, data in enumerate(self.raw_rpe_dyn):
            t = data["time"]
            dyn_trans = data["RPE_trans"]
            dyn_rot = data["RPE_rot"]
            orb_label = data["label"]
            orb_plotstyle = data["plotstyle"]
            plt.figure("RPE Magnitude over distance")
            plt.subplot(2, 1, 1)
            plt.plot(t, dyn_trans, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("translational error [-]")

            plt.subplot(2, 1, 2)
            plt.plot(t, dyn_rot, orb_plotstyle, label=orb_label)
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

    Test = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
    pdb.set_trace()
    Test.ShowRpeDistAll()

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