from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
import pdb
import time

def main():
    """Describes the performance of a certain scenario in a certain starting location"""
    Towns = (1, 2, 3)
    ds = 20
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    scenario_performance_data = []
    for Town in Towns:
        if Town == 1:
            starting_locations = (27, 0, 58)
        elif Town == 2:
            starting_locations = (37, 78)
        elif Town == 3:
            starting_locations = (75, 97, 127, 132)
        else:
            print("Town does not exist")
            return
        for SL in starting_locations:
            dir_name_stat = DirName(Town, SL, "static")
            dir_name_dyn = DirName(Town, SL, "dynamic", ds)

            orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat)
            orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn)
            print("Converting T{}SL{}".format(Town, SL))
            Test = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
            scenario_performance_data.append(Test)

    A = ScenarioPerformance(scenario_performance_data)
    pdb.set_trace()


class ScenarioPerformance:
    def __init__(self, list_scenario_performance):
        self.scenario = list_scenario_performance[0].scenario

        total_filtered_static = 0.0
        total_filtered_dynamic = 0.0

        total_lost_track_static = 0.0
        total_lost_track_dynamic = 0.0

        total_false_loop_static = 0.0
        total_false_loop_dynamic = 0.0

        total_rmse_static_trans = 0.0
        total_rmse_static_rot = 0.0

        total_rmse_dynamic_trans = 0.0
        total_rmse_dynamic_rot = 0.0

        for data in list_scenario_performance:
            total_lost_track_static += data.lost_track_static
            total_false_loop_static += data.false_loop_static
            total_filtered_static += data.ratio_filtered_static
            total_rmse_static_trans += data.rmse_static_avg[0]
            total_rmse_static_rot += data.rmse_static_avg[1]

            total_lost_track_dynamic += data.lost_track_dynamic
            total_false_loop_dynamic += data.false_loop_dynamic
            total_filtered_dynamic += data.ratio_filtered_dynamic
            total_rmse_dynamic_trans += data.rmse_dynamic_avg[0]
            total_rmse_dynamic_rot += data.rmse_dynamic_avg[1]

        self.avg_filtered_static = total_filtered_static/len(list_scenario_performance)
        self.avg_false_loop_static = total_false_loop_static/len(list_scenario_performance)
        self.avg_lost_track_static = total_lost_track_static/len(list_scenario_performance)
        self.avg_rmse_static = (total_rmse_static_trans/len(list_scenario_performance),
                                total_rmse_static_rot/len(list_scenario_performance))

        self.avg_filtered_dynamic = total_filtered_dynamic/len(list_scenario_performance)
        self.avg_false_loop_dynamic = total_false_loop_dynamic/len(list_scenario_performance)
        self.avg_lost_track_dynamic = total_lost_track_dynamic/len(list_scenario_performance)
        self.avg_rmse_dynamic = (total_rmse_dynamic_trans/len(list_scenario_performance),
                                total_rmse_dynamic_rot/len(list_scenario_performance))

        stat_vs_dyn_trans = (self.avg_rmse_dynamic[0] - self.avg_rmse_static[0]) / self.avg_rmse_static[0] * 100.0
        stat_vs_dyn_rot = (self.avg_rmse_dynamic[1] - self.avg_rmse_static[1]) / self.avg_rmse_static[1] * 100.0
        self.avg_static_vs_dynamic = (stat_vs_dyn_trans, stat_vs_dyn_rot)


    def SummaryPerformance(self):
        """Prints a summary of the performance of the scenario and location"""

        print("PERFORMANCE SUMMARY \nScenario: {} - {}m \n".format(self.scenario["Scenario"],
                                                                   self.scenario["Distance"]))
        print("Dynamic scenario\n"
              "----------------\n"
              "Filtered out:                    {}\n"
              "Track failure:                   {}\n"
              "False loops:                     {}\n"
              "Increase error wrt static [%]:   {}\n"
              "RMSE RPE:                        {}\n".format(self.avg_filtered_dynamic,
                                                             self.avg_lost_track_dynamic,
                                                             self.avg_false_loop_dynamic,
                                                             self.avg_static_vs_dynamic,
                                                             self.avg_rmse_dynamic))
        print("Static scenario\n"
              "----------------\n"
              "Filtered out:                    {}\n"
              "Track failure:                   {}\n"
              "False loops:                     {}\n"
              "RMSE RPE:                        {}\n".format(self.avg_filtered_static,
                                                             self.avg_lost_track_static,
                                                             self.avg_false_loop_static,
                                                             self.avg_rmse_static))



if __name__ == "__main__":
    main()