from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
import pdb
import time

def main():
    """Describes the performance of a certain scenario in a certain starting location"""
    Towns = (1, 2, 3)
    ds = 15
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    scenario_performance_data = []
    for Town in Towns:
        if Town == 1:
            starting_locations = (0, 27, 58)
        elif Town == 2:
            starting_locations = (18, 37, 78)
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
    #A.CreateLatexTable(scenario_performance_data)
    # A.SummaryPerformance()
    print(A.LatexTableStatic)
    print(A.LatexTableDynamic)
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

        total_static_vs_dynamic_trans = 0.0
        total_static_vs_dynamic_rot = 0.0

        total_rmse_static_trans = 0.0
        total_rmse_static_rot = 0.0

        total_rmse_dynamic_trans = 0.0
        total_rmse_dynamic_rot = 0.0

        nr_all_data_filtered = 0
        for data in list_scenario_performance:
            total_lost_track_static += data.lost_track_static
            total_false_loop_static += data.false_loop_static
            total_filtered_static += data.ratio_filtered_static
            # Is it fair to have an unweighted average of the performance value, when not all performance value have
            # the same amount of data points.
            # total_rmse_static_trans += data.rmse_static_avg[0]
            # total_rmse_static_rot += data.rmse_static_avg[1]

            total_lost_track_dynamic += data.lost_track_dynamic
            total_false_loop_dynamic += data.false_loop_dynamic
            total_filtered_dynamic += data.ratio_filtered_dynamic
            # total_rmse_dynamic_trans += data.rmse_dynamic_avg[0]
            # total_rmse_dynamic_rot += data.rmse_dynamic_avg[1]

            if data.static_vs_dynamic_avg == None:
                nr_all_data_filtered += 1
            else:
                total_static_vs_dynamic_trans += data.static_vs_dynamic_avg[0]
                total_static_vs_dynamic_rot += data.static_vs_dynamic_avg[1]

        self.avg_filtered_static = total_filtered_static/len(list_scenario_performance)
        self.avg_false_loop_static = total_false_loop_static/len(list_scenario_performance)
        self.avg_lost_track_static = total_lost_track_static/len(list_scenario_performance)
        # self.avg_rmse_static = (total_rmse_static_trans/len(list_scenario_performance),
        #                        total_rmse_static_rot/len(list_scenario_performance))

        self.avg_filtered_dynamic = total_filtered_dynamic/len(list_scenario_performance)
        self.avg_false_loop_dynamic = total_false_loop_dynamic/len(list_scenario_performance)
        self.avg_lost_track_dynamic = total_lost_track_dynamic/len(list_scenario_performance)
        # self.avg_rmse_dynamic = (total_rmse_dynamic_trans/len(list_scenario_performance),
        #                        total_rmse_dynamic_rot/len(list_scenario_performance))

        # take the average of increase error, but take into account the locations when all dynamic data is filtered out.
        self.avg_static_vs_dynamic = (total_static_vs_dynamic_trans /
                                      (len(list_scenario_performance)-nr_all_data_filtered),
                                      total_static_vs_dynamic_rot /
                                      (len(list_scenario_performance)-nr_all_data_filtered))
        self.LatexTableStatic = ""
        self.LatexTableDynamic = ""

        self.CreateLatexTable(list_scenario_performance)

    def CreateLatexTable(self, list_scenario_performance):
        """Creates table for latex purposes"""

        # general format of table
        format_table = r"\begin{table}[h!]" + "\n" + r"\centering" + "\n" + \
                       r"\begin{tabular}{m{1cm} m{5mm}|m{15mm}|m{15mm}|m{15mm}|m{15mm}|m{15mm}|m{20mm}} \hline" + "\n"

        # generate static table
        title_static = r"\multicolumn{8}{c}{Static}\\\hline\hline" + "\n"
        title_dynamic = "\\multicolumn{{8}}{{c}}{{Dynamic: {} - distance: {} m}}\\\\\\hline\\hline".format(self.scenario['Scenario'], self.scenario['Distance']) + "\n"

        # strings in column
        string_columns = r"Map & Nr & \multicolumn{2}{c|}{RMSE RPE trans. [-]} & " \
                         r"\multicolumn{2}{c|}{RMSE RPE rot. [deg/m]} &  &  \\" \
                         r"& & \multicolumn{1}{c}{Average} & \multicolumn{1}{c|}{Variance} & " \
                         r"\multicolumn{1}{c}{Average} & \multicolumn{1}{c|}{Variance} & " \
                         r"tracking \newline failure [\%]& false loop closure [\%] \\\hline" + "\n"

        # static data town
        table_content_static = ""
        table_content_dynamic = ""
        for i in range(10):
            row_data = ""
            if i == 0:
                town_column = r"\multirow{3}{4em}{Town01}"
            elif i == 3:
                town_column = r"\hline \multirow{3}{4em}{Town02}"
            elif i == 6:
                town_column = r"\hline \multirow{3}{4em}{Town03}"
            else:
                town_column = ""
            location_data = list_scenario_performance[i]
            table_data_static = "& {0:d} & {1:.3g} & {2:.3g} & {3:.3g} & {4:.3g} & {5:.1f} & {6:.1f}"
            row_data_static = town_column + table_data_static.format(i, location_data.rmse_static_avg[0],
                                                       location_data.rmse_static_var[0],
                                                       location_data.rmse_static_avg[1],
                                                       location_data.rmse_static_var[1],
                                                       location_data.lost_track_static,
                                                       location_data.false_loop_static) + r"\\" + "\n"

            table_data_dynamic = "& {0:d} & {1:.3g} & {2:.3g} & {3:.3g} & {4:.3g} & {5:.1f} & {6:.1f}"
            row_data_dynamic = town_column + table_data_dynamic.format(i, location_data.rmse_dynamic_avg[0],
                                                              location_data.rmse_dynamic_var[0],
                                                              location_data.rmse_dynamic_avg[1],
                                                              location_data.rmse_dynamic_var[1],
                                                              location_data.lost_track_dynamic,
                                                              location_data.false_loop_dynamic) + r"\\" + "\n"


            table_content_static += row_data_static
            table_content_dynamic += row_data_dynamic

        # finish table
        end_table_static = r"\end{tabular}" + "\n" + r"\caption{Performance ORB SLAM in static environment}" + "\n" + r"\label{tab:RmseRpeStaticAll}" + "\n" + r"\end{table}"
        end_table_dynamic = r"\end{tabular}" + "\n" + r"\caption{Performance ORB SLAM in dynamic environment}" + "\n" + r"\label{tab:RmseRpeDynamicAll}" + "\n" + r"\end{table}"
        final_table_static = format_table + title_static + string_columns + table_content_static + end_table_static
        final_table_dynamic = format_table + title_dynamic + string_columns + table_content_dynamic + end_table_dynamic

        self.LatexTableStatic = final_table_static
        self.LatexTableDynamic = final_table_dynamic

    def SummaryPerformance(self):
        """Prints a summary of the performance of the scenario and location"""

        print("PERFORMANCE SUMMARY \nScenario: {} - {}m \n".format(self.scenario["Scenario"],
                                                                   self.scenario["Distance"]))
        print("Dynamic scenario\n"
              "----------------\n"
              "Filtered out:                    {}\n"
              "Track failure:                   {}\n"
              "False loops:                     {}\n"
              "Increase error wrt static [%]:   {}\n".format(self.avg_filtered_dynamic,
                                                             self.avg_lost_track_dynamic,
                                                             self.avg_false_loop_dynamic,
                                                             self.avg_static_vs_dynamic))
        print("Static scenario\n"
              "----------------\n"
              "Filtered out:                    {}\n"
              "Track failure:                   {}\n"
              "False loops:                     {}\n".format(self.avg_filtered_static,
                                                             self.avg_lost_track_static,
                                                             self.avg_false_loop_static))



if __name__ == "__main__":
    main()