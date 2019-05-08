from func_EvaluateRpeDist import evaluate_RPE_dist, calc_rmse
from matplotlib import pyplot as plt
import numpy as np
import pdb


class ScenarioLocationPerformance:
    def __init__(self, ds, Town, SL, slam_stat, slam_dyn, gt):
        # description of scenario
        self.scenario = {"Scenario": "Stuck behind van", "Distance": ds}
        self.location = {"Town": Town, "Location": SL}
        # contains the list of all crs objects static
        self.slam_stat = []
        # contains list of all crs objects dynamic
        self.slam_dyn = []

        # raw RPE dist data in a dict form. Everything is here to make a plot that explains everything
        self.raw_rpe_stat = []
        self.raw_rpe_dyn = []

        # filtered RPE dist data in dict form. All failed tracking and false loop closure are removed
        self.filtered_rpe_stat = []
        self.filtered_rpe_dyn = []

        # Root mean square error translational and rotational dynamic and static of each filtered sequence
        self.rmse_static = []
        self.rmse_dynamic = []

        # ratio that lost track of location
        self.lost_track_static = ()
        self.lost_track_dynamic = ()

        # ratio that had false loop closure
        # NOTE: THIS DETECTS ANY LOOP CLOSURES, NOT FALSE LOOP CLOSURES
        self.false_loop_static = ()
        self.false_loop_dynamic = ()

        # total ratio that is filtered out due to false loop closure or lost track
        self.ratio_filtered_static = ()
        self.ratio_filtered_dynamic = ()

        # average and variance root mean square error translational and rotational component, static and dynamic
        self.rmse_static_avg = ()
        self.rmse_static_var = ()
        self.rmse_dynamic_avg = ()
        self.rmse_dynamic_var = ()

        # Percentage increase error dynamic over static
        self.static_vs_dynamic_avg = ()

        # save the crf data in lists
        for orb in slam_stat:
            self.slam_stat.append(orb)

        for orb in slam_dyn:
            self.slam_dyn.append(orb)

        # get the RPE performance from this data
        self.SaveRpeData(gt)

        # find and filter all data that loose track during orb estimation
        filter_index_stat, filter_index_dyn = self.FilterSuccesfullTracking(gt)
        tracked_raw_rpe_stat = [self.raw_rpe_stat[index] for index in filter_index_stat]
        tracked_raw_rpe_dyn = [self.raw_rpe_dyn[index] for index in filter_index_dyn]
        tracked_slam_stat = [self.slam_stat[index] for index in filter_index_stat]
        tracked_slam_dyn = [self.slam_dyn[index] for index in filter_index_dyn]

        # find and filter all data that have false loop closure and store data in class
        correct_loop_dyn = self.FilterFalseLoopClosure(tracked_slam_dyn)
        self.ratio_filtered_dynamic = 1.0 - float(len(correct_loop_dyn))/float(len(self.slam_dyn))
        self.false_loop_dynamic = self.ratio_filtered_dynamic - self.lost_track_dynamic

        correct_loop_stat = self.FilterFalseLoopClosure(tracked_slam_stat)
        self.ratio_filtered_static = 1.0 - float(len(correct_loop_stat))/float(len(self.slam_stat))
        self.false_loop_static = self.ratio_filtered_static - self.lost_track_static

        self.filtered_rpe_stat = [tracked_raw_rpe_stat[index] for index in correct_loop_stat]
        self.filtered_rpe_dyn = [tracked_raw_rpe_dyn[index] for index in correct_loop_dyn]

        # calculate Root Mean Square Error of the filtered data
        for filtered_rpe_data in self.filtered_rpe_stat:
            trans_errors = filtered_rpe_data["RPE_trans"]
            rot_errors = filtered_rpe_data["RPE_rot"]
            orb_rmse = (calc_rmse(trans_errors), calc_rmse(rot_errors))
            self.rmse_static.append(orb_rmse)

        for filtered_rpe_data in self.filtered_rpe_dyn:
            trans_errors = filtered_rpe_data["RPE_trans"]
            rot_errors = filtered_rpe_data["RPE_rot"]
            orb_rmse = (calc_rmse(trans_errors), calc_rmse(rot_errors))
            self.rmse_dynamic.append(orb_rmse)

        # calculate average performance of the filtered data
        self.CalculateAverageRMSE()

        # compare static performance vs dynamic performance
        self.CompareAvgStaticVsDynamic()



    def SummaryPerformance(self):
        """Prints a summary of the performance of the scenario and location"""

        print("PERFORMANCE SUMMARY \nTown {}, Starting location {}, Scenario: {} - {}m \n".format(self.location["Town"], self.location["Location"], self.scenario["Scenario"], self.scenario["Distance"]))
        print("Dynamic scenario\n"
              "----------------\n"
              "Filtered out:                    {}\n"
              "Track failure:                   {}\n"
              "False loops:                     {}\n"
              "Increase error wrt static [%]:   {}\n"
              "RMSE RPE:                        {}\n".format(self.ratio_filtered_dynamic,
                                                       self.lost_track_dynamic,
                                                       self.false_loop_dynamic,
                                                       self.static_vs_dynamic_avg,
                                                       self.rmse_dynamic_avg))
        print("Static scenario\n"
              "----------------\n"
              "Filtered out:                    {}\n"
              "Track failure:                   {}\n"
              "False loops:                     {}\n"
              "RMSE RPE:                        {}\n".format(self.ratio_filtered_static,
                                                             self.lost_track_static,
                                                             self.false_loop_static,
                                                             self.rmse_static_avg))

    def CompareAvgStaticVsDynamic(self):
        if self.rmse_dynamic_avg == None or self.rmse_static_avg == None:
            self.static_vs_dynamic_avg = None
        else:
            stat_vs_dyn_trans = (self.rmse_dynamic_avg[0] - self.rmse_static_avg[0] )/self.rmse_static_avg[0]*100.0
            stat_vs_dyn_rot = (self.rmse_dynamic_avg[1] - self.rmse_static_avg[1])/self.rmse_static_avg[1]*100.0
            self.static_vs_dynamic_avg = (stat_vs_dyn_trans, stat_vs_dyn_rot)

    def CalculateAverageRMSE(self):
        """Calculate the average and variance of starting location"""

        if self.ratio_filtered_static == 1.0:
            self.rmse_static_avg = None
            self.rmse_static_var = None
        else:
            rmse_trans_static = np.array([rmse[0] for rmse in self.rmse_static])
            rmse_rot_static = np.array([rmse[1] for rmse in self.rmse_static])
            self.rmse_static_avg = (np.mean(rmse_trans_static), np.mean(rmse_rot_static))
            self.rmse_static_var = (np.var(rmse_trans_static), np.var(rmse_rot_static))

        if self.ratio_filtered_dynamic == 1.0:
            self.rmse_dynamic_avg = None
            self.rmse_dynamic_var = None
        else:
            rmse_trans_dynamic = np.array([rmse[0] for rmse in self.rmse_dynamic])
            rmse_rot_dynamic = np.array([rmse[1] for rmse in self.rmse_dynamic])
            self.rmse_dynamic_avg = (np.mean(rmse_trans_dynamic), np.mean(rmse_rot_dynamic))
            self.rmse_dynamic_var = (np.var(rmse_trans_dynamic), np.var(rmse_rot_dynamic))


    def SaveRpeData(self, gt):
        """Calculate the RPE dist for all sequences and the RMSE"""
        for orb in self.slam_stat:
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb, 100)
            RawRpeSingle = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors,
                            "plotstyle": orb.plotstyle, "label": orb.label}
            self.raw_rpe_stat.append(RawRpeSingle)


        for orb in self.slam_dyn:
            time_used, trans_errors, rot_errors = evaluate_RPE_dist(gt, orb, 100)
            RawRpeSingle = {"time": time_used, "RPE_trans": trans_errors, "RPE_rot": rot_errors,
                            "plotstyle": orb.plotstyle, "label": orb.label}
            self.raw_rpe_dyn.append(RawRpeSingle)


    def FilterFalseLoopClosure(self, tracked_slam):
        """outputs the indices that has succesfully tracked a full trajectory without false loop closing"""
        # Threshold: if the estimated travelled distance is bigger than 2m in one time step,
        # than false loop closure is detected
        threshold = 2
        all_travels = {}
        correct_loop_index = []
        for index_orb, orb in enumerate(tracked_slam):
            travels = []
            for index, position in enumerate(orb.positions[:-2]):
                travelled_step = np.linalg.norm(orb.positions[index+1]-position)
                travels.append(travelled_step)
                if travelled_step > threshold:
                    # false_loop_index.append(index_orb)
                    break
                if index == len(orb.positions[:-3]):
                    correct_loop_index.append(index_orb)

            all_travels[index_orb] = travels

        return correct_loop_index

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

    def ShowRpeDistFiltered(self):
        for index, static_data in enumerate(self.filtered_rpe_stat):
            t = static_data["time"]
            stat_trans = static_data["RPE_trans"]
            stat_rot = static_data["RPE_rot"]
            orb_label = static_data["label"]
            orb_plotstyle = static_data["plotstyle"]
            plt.figure("RPE Magnitude over distance, filtered")
            plt.subplot(2, 1, 1)
            plt.plot(t, stat_trans, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("translational error [-]")

            plt.subplot(2, 1, 2)
            plt.plot(t, stat_rot, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("rotational error [deg/m]")
            plt.legend()

        for index, data in enumerate(self.filtered_rpe_dyn):
            t = data["time"]
            dyn_trans = data["RPE_trans"]
            dyn_rot = data["RPE_rot"]
            orb_label = data["label"]
            orb_plotstyle = data["plotstyle"]
            plt.figure("RPE Magnitude over distance, filtered")
            plt.subplot(2, 1, 1)
            plt.plot(t, dyn_trans, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("translational error [-]")

            plt.subplot(2, 1, 2)
            plt.plot(t, dyn_rot, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("rotational error [deg/m]")
            plt.legend()


    def ShowRpeDistAll(self):
        """Plots the RPE dist of all the data that is used to make a Scenario Location specific performance class"""
        for index, static_data in enumerate(self.raw_rpe_stat):
            t = static_data["time"]
            stat_trans = static_data["RPE_trans"]
            stat_rot = static_data["RPE_rot"]
            orb_label = static_data["label"]
            orb_plotstyle = static_data["plotstyle"]
            plt.figure("RPE Magnitude over distance, unfiltered")
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
            plt.figure("RPE Magnitude over distance, unfiltered")
            plt.subplot(2, 1, 1)
            plt.plot(t, dyn_trans, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("translational error [-]")

            plt.subplot(2, 1, 2)
            plt.plot(t, dyn_rot, orb_plotstyle, label=orb_label)
            plt.xlabel("time [s]")
            plt.ylabel("rotational error [deg/m]")
            plt.legend()
