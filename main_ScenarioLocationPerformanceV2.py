from class_ScenarioLocationPerformance import *
from main_InspectData import DirName, InspectJsonFileInDir
from uncertainties import ufloat
import pdb
import time

def main_stuckbehindvan():
    """Describes the error increase wrt static for each dynamic scenario """
    Towns = (1, 2)
    dynamic_scenarios = (20, 15, 10)
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"
    # base_dir = "/media/svanschouwenburg/Elements/stuckbehindvan/20fps/"

    scenario_performance_data = []
    for Town in Towns:
        if Town == 1:
            starting_locations = (0, 27, 58)
        elif Town == 2:
            starting_locations = (37, 78)
        elif Town == 3:
            # starting_locations = (75, 97, 127, 132)
            starting_locations = [127]
        else:
            print("Town does not exist")
            return
        for SL in starting_locations:
            for ds in dynamic_scenarios:
                dir_name_stat = DirName(Town, SL, "static")
                dir_name_dyn = DirName(Town, SL, "dynamic", ds)

                orb_static, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_stat, "VO")
                orb_dynamic, gt = InspectJsonFileInDir(Town, SL, base_dir, dir_name_dyn, "VO")
                print("Converting T{}SL{}".format(Town, SL))
                Test = ScenarioLocationPerformance(ds, Town, SL, orb_static, orb_dynamic, gt)
                scenario_performance_data.append(Test)
    A = ScenarioPerformance(scenario_performance_data)

def main_vansoppositeroad():
    Towns = (1, 2)
    dynamic_variable = 10
    base_dir = "/home/sietse/results_carla0.9/VansOppositeRoad/"



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


        self.avg_filtered_dynamic = total_filtered_dynamic/len(list_scenario_performance)
        self.avg_false_loop_dynamic = total_false_loop_dynamic/len(list_scenario_performance)
        self.avg_lost_track_dynamic = total_lost_track_dynamic/len(list_scenario_performance)

        dict_list = self.PerformanceLocation_func(list_scenario_performance)
        self.CreateLatexTableDiscussionTrans(dict_list)
        self.CreateLatexTableDiscussionRot(dict_list)


    def PerformanceLocation_func(self, list_scenario_performance):
        """returns list of dictionaries that contain performance in each location over different dynamic scenarios """

        dict_list = []

        for data in list_scenario_performance:
            town = data.location["Town"]
            location = data.location["Location"]
            location_string = "Town {} Location {}".format(town, location)
            is_in_dict = False
            print(location_string)
            print(data.scenario["Distance"])
            # check if this location has already a dictionary
            for dict in dict_list:
                if dict["Location"] == location_string:
                    is_in_dict = True

            # if it is not in dictionary, a dictionary has to be made an put in the list
            if not is_in_dict:
                # print("is not in dictionary")
                new_dict = {"Location": location_string,
                            "Static_trans": ufloat(data.rmse_static_avg[0], data.rmse_static_std[0]),
                            "Static_rot": ufloat(data.rmse_static_avg[1], data.rmse_static_std[1])}
                dict_list.append(new_dict)

            # Now the location is definitely in the list
            # Now this specific scenario is added to the dictionary
            for dict in dict_list:
                # get the right dictionary from the list
                if dict["Location"] == location_string:
                    # add the right inter vehicle distance to the dictionary
                    # get error propagation in the calculation
                    ds = data.scenario["Distance"]
                    trans_dynamic = ufloat(data.rmse_dynamic_avg[0], data.rmse_dynamic_std[0])
                    trans_static = ufloat(data.rmse_static_avg[0], data.rmse_static_std[0])
                    dict["d{}_trans".format(ds)] = trans_dynamic
                    dict["d{}_trans_per".format(ds)] = (trans_dynamic - trans_static) / trans_static.nominal_value * 100.0

                    rot_dynamic = ufloat(data.rmse_dynamic_avg[1], data.rmse_dynamic_std[1])
                    dict["d{}_rot".format(ds)] = rot_dynamic
                    rot_static = ufloat(data.rmse_static_avg[1], data.rmse_static_std[1])
                    dict["d{}_rot_per".format(ds)] = (rot_dynamic - rot_static) / rot_static.nominal_value * 100.0
                    dict["d{}_filteredout".format(ds)] = data.ratio_filtered_dynamic
                    dict["d{}_falseloop".format(ds)] = data.false_loop_dynamic
                    dict["d{}_failedtrack".format(ds)] = data.lost_track_dynamic

        return dict_list

    def CreateLatexTableDiscussionTrans(self, dict_list):
        format_table = r"\begin{table}[h!] " + "\n"\
                       r"\centering " + "\n"\
                       r"\begin{tabular*}\columnwidth{@{\extracolsep{\fill}}l|c|c|c|c|c}" + "\n"

        title = r"\multicolumn{6}{c}{RMSE Trans wrt static} \\\hline \hline"

        string_columns = r" Map & Static & \multicolumn{2}{c|}{Dist: 20 m} &  \multicolumn{2}{c}{Dist: 15 m} \\ " \
                         r"& [-] & \multicolumn{1}{c}{[-]} & [\%] &\multicolumn{1}{c}{[-]} & [\%] \\ \hline" +"\n"
        table_content = ""

        for dict in dict_list:

            location_string = dict["Location"]
            location_nr = location_string.split(" ")[-1]
            town_nr = location_string.split(" ")[1]
            if town_nr == '1':
                if location_nr == '0':
                    traj_index = 1
                elif location_nr == '27':
                    traj_index = 2
                elif location_nr == '58':
                    traj_index = 3
                else:
                    print("Town 1 does not have this SL")
                    return
            elif town_nr == '2':
                if location_nr == '18':
                    traj_index = 1
                elif location_nr == '37':
                    traj_index = 2
                elif location_nr == '78':
                    traj_index = 3
                else:
                    print("Town 2 does not have this SL")
                    return

            elif town_nr == '3':
                if location_nr == '75':
                    traj_index = 1
                elif location_nr == '97':
                    traj_index = 2
                elif location_nr == '127':
                    traj_index = 3
                elif location_nr == '132':
                    traj_index = 4
                else:
                    print("Town 3 does not have this SL")
                    return
            else:
                print("Combo town and SL does not exist")
                return

            column0 = "T{}Nr{}".format(town_nr, traj_index)
            dic_data = "& ${:.3f} \pm {:.3f}$ & ${:.3f} \pm {:.3f}$ & ${:.1f} \pm {:.1f}$ & ${:.3f} \pm {:.3f}$ & ${:.1f} \pm {:.1f}$".format(dict["Static_trans"].nominal_value, 
							 dict["Static_trans"].std_dev, 
                                                         dict["d20_trans"].nominal_value, 
							 dict["d20_trans"].std_dev, 
                                                         dict["d20_trans_per"].nominal_value,
							 dict["d20_trans_per"].std_dev,
                                                         dict["d15_trans"].nominal_value,
							 dict["d15_trans"].std_dev,
                                                         dict["d15_trans_per"].nominal_value, 
							 dict["d15_trans_per"].std_dev)
            table_data_row = column0 + dic_data + r"\\" + "\n"
            table_content += table_data_row

        end_table = r"\end{tabular*} " + "\n"\
                    r"\caption{Caption}" + "\n"\
                    r"\label{tab:my_label}" + "\n"\
                    r"\end{table}"
        final_table = format_table + title + string_columns + table_content + end_table

        print(final_table)

    def CreateLatexTableDiscussionRot(self, dict_list):
        format_table = r"\begin{table}[h!] " + "\n" \
                        r"\centering " + "\n" \
                        r"\begin{tabular*}\columnwidth{@{\extracolsep{\fill}}l|c|c|c|c|c}" + "\n"

        title = r"\multicolumn{6}{c}{RMSE Rot wrt static} \\\hline \hline"

        string_columns = r" Map & Static & \multicolumn{2}{c|}{Dist: 20 m} &  \multicolumn{2}{c}{Dist: 15 m} \\ " \
                         r"& [deg/m] & \multicolumn{1}{c}{[deg/m]} & [\%] &\multicolumn{1}{c}{[deg/m]} & [\%] \\ \hline" + "\n"
        table_content = ""

        for dict in dict_list:

            location_string = dict["Location"]
            location_nr = location_string.split(" ")[-1]
            town_nr = location_string.split(" ")[1]
            if town_nr == '1':
                if location_nr == '0':
                    traj_index = 1
                elif location_nr == '27':
                    traj_index = 2
                elif location_nr == '58':
                    traj_index = 3
                else:
                    print("Town 1 does not have this SL")
                    return
            elif town_nr == '2':
                if location_nr == '18':
                    traj_index = 1
                elif location_nr == '37':
                    traj_index = 2
                elif location_nr == '78':
                    traj_index = 3
                else:
                    print("Town 2 does not have this SL")
                    return

            elif town_nr == '3':
                if location_nr == '75':
                    traj_index = 1
                elif location_nr == '97':
                    traj_index = 2
                elif location_nr == '127':
                    traj_index = 3
                elif location_nr == '132':
                    traj_index = 4
                else:
                    print("Town 3 does not have this SL")
                    return
            else:
                print("Combo town and SL does not exist")
                return

            column0 = "T{}Nr{}".format(town_nr, traj_index)
            dic_data = "& ${:.3f} \pm {:.3f}$ & ${:.3f} \pm {:.3f}$ & ${:.1f} \pm {:.1f}$ & ${:.3f} \pm {:.3f}$ & ${:.1f} \pm {:.1f}$".format(dict["Static_rot"].nominal_value, 
							 dict["Static_rot"].std_dev, 
                                                         dict["d20_rot"].nominal_value, 
							 dict["d20_rot"].std_dev, 
                                                         dict["d20_rot_per"].nominal_value,
							 dict["d20_rot_per"].std_dev,
                                                         dict["d15_rot"].nominal_value,
							 dict["d15_rot"].std_dev,
                                                         dict["d15_rot_per"].nominal_value, 
							 dict["d15_rot_per"].std_dev)
            table_data_row = column0 + dic_data + r"\\" + "\n"
            table_content += table_data_row

        end_table = r"\end{tabular*} " + "\n" \
                    r"\caption{Caption}" + "\n" \
                    r"\label{tab:my_label}" + "\n" \
                    r"\end{table}"
        final_table = format_table + title + string_columns + table_content + end_table

        print(final_table)

if __name__ == "__main__":
    main()
