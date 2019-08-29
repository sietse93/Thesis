import numpy as np
import transformations
from pyquaternion import Quaternion
from class_ConvertRefFrame import myround
from func_Convert2Json import crf2json
import math
import pdb

base_dir = "/home/sietse/results_carla0.9"

def main():
    # Should convert a txt file to a type of ConvertRefFrame class which tells you under what conditions the map was created
    # and which scenario is used for localization only mode.

    # Each txt file consists of 6 trajectory estimations.
    # 1) is the mapping process
    # 2) is localization only in the scenario of the mapping process
    # 3) 4) are from one scenario
    # 5) 6) are from the other scenario
    # Always use the 2nd option in a certain scenario, since the first time you run a rosbag it tends to be laggy.

    # Name of the file first gives the conditions of the map,
    # Next the conditions of the first scenario localization mode on and the 2nd scenario localization mode on.

    town = 1
    mapping_sequence = ["StatSc1Sc2", "Sc1StatSc2", "Sc2StatSc1"]
    starting_locations = (0, 27, 58)
    orb_numbers = range(5)
    for seq in mapping_sequence:
        for SL in starting_locations:
            for orb_nr in orb_numbers:
                if seq[0:4] == "Stat":
                    base_folder = "/stuckbehindvan/20fps"
                    file = "/T{}_SL{}_s/T{}_SL{}_s_orb_LM_{}_{}.txt".format(town, SL, town, SL, seq, orb_nr)
                elif seq[0:3] == "Sc1":
                    base_folder = "/stuckbehindvan/20fps"
                    file = "/T{}_SL{}_d15/T{}_SL{}_d15_orb_LM_{}_{}.txt".format(town, SL, town, SL, seq, orb_nr)

                elif seq[0:3] == "Sc2":
                    base_folder = "/VansOppositeRoad"
                    file = "/T{}_SL{}_d10/T{}_SL{}_d10_orb_LM_{}_{}.txt".format(town, SL, town, SL, seq, orb_nr)
                else:
                    print("PROBLEM")
                    return

                # something went wrong in this mapping process. so data is not usable
                if seq == "Sc1StatSc2" and SL == 0 and orb_nr == 2:
                    continue
                if seq == "Sc1StatSc2" and SL == 58 and orb_nr == 2:
                    continue

                if seq == "Sc2StatSc1" and SL == 58 and orb_nr == 1:
                    continue

                if seq == "Sc2StatSc1" and SL == 58 and orb_nr == 4:
                    continue

                f_loc = base_dir+base_folder+file
                txt_data = open(f_loc, "r")

                time = []
                x = []
                y = []
                z = []
                q1 = []
                q2 = []
                q3 = []
                q4 = []

                # read all the data!!
                for line_data in txt_data:
                    line = line_data.split(" ")
                    float_line = [float(element) for element in line]
                    time.append(float_line[0])
                    x.append(float_line[1])
                    y.append(float_line[2])
                    z.append(float_line[3])
                    q1.append(float_line[4])
                    q2.append(float_line[5])
                    q3.append(float_line[6])
                    q4.append(float_line[7])

                # Find out where the new rosbag starts
                new_slam_index = []
                t_diff_list = []
                for index, t in enumerate(time[1::]):
                    t_diff = abs(t-time[index])
                    t_diff_list.append(t_diff)
                    if t_diff > 115.0:
                        new_slam_index.append(index)

                # Based on file name it should classify what was the base map and what is the scenario. This should be described
                # in the class so the json file has the correct naming.

                file_nr = file[-5]
                scenario_file_name = file.split("_")[-2]

                if scenario_file_name[0:2] != "Sc":
                    # the map was made in static conditions.
                    sc0_string = scenario_file_name[0:4]
                    sc1_string = scenario_file_name[4:7]
                    sc2_string = scenario_file_name[7::]

                else:
                    sc0_string = scenario_file_name[0:3]
                    # Note: Make sure Stat is always the 2nd tested scenario so file name should be either
                    # "Sc1StatSc2" or "Sc2StatSc1"
                    sc1_string = scenario_file_name[3:7]
                    sc2_string = scenario_file_name[7::]

                # data is going to be divided into four categories:
                # 1) localization and mapping mode (check if using a map is better)
                # 2) localization mode only, same scenario as 1)
                # 3) localization mode only, different scenario as 1)
                # 4) localization mode only, different scenario as 1) and 3)

                # But for 1) you can use json files of previous experiments.
                # variable name has a numbering system based on the name of the file (e.g. StatSc1Sc2)
                # stat = 0, sc1 = 1, sc2 = 2

                # 2) localization mode only, same scenario as 1)
                time_lm_0 = time[new_slam_index[0]+1:new_slam_index[1]+1]
                x_lm_0 = x[new_slam_index[0]+1:new_slam_index[1]+1]
                y_lm_0 = y[new_slam_index[0]+1:new_slam_index[1]+1]
                z_lm_0 = z[new_slam_index[0]+1:new_slam_index[1]+1]
                q1_lm_0 = q1[new_slam_index[0]+1:new_slam_index[1]+1]
                q2_lm_0 = q2[new_slam_index[0]+1:new_slam_index[1]+1]
                q3_lm_0 = q3[new_slam_index[0]+1:new_slam_index[1]+1]
                q4_lm_0 = q4[new_slam_index[0]+1:new_slam_index[1]+1]
                label_lm_0 = sc0_string + sc0_string

                #  3) localization mode only, different scenario as 1)
                time_lm_1 = time[new_slam_index[2]+1:new_slam_index[3]+1]
                x_lm_1 = x[new_slam_index[2]+1:new_slam_index[3]+1]
                y_lm_1 = y[new_slam_index[2]+1:new_slam_index[3]+1]
                z_lm_1 = z[new_slam_index[2]+1:new_slam_index[3]+1]
                q1_lm_1 = q1[new_slam_index[2]+1:new_slam_index[3]+1]
                q2_lm_1 = q2[new_slam_index[2]+1:new_slam_index[3]+1]
                q3_lm_1 = q3[new_slam_index[2]+1:new_slam_index[3]+1]
                q4_lm_1 = q4[new_slam_index[2]+1:new_slam_index[3]+1]
                label_lm_1 = sc0_string + sc1_string

                # 4) localization mode only, different scenario as 1) and 3)
                time_lm_2 = time[new_slam_index[4]::]
                x_lm_2 = x[new_slam_index[4]::]
                y_lm_2 = y[new_slam_index[4]::]
                z_lm_2 = z[new_slam_index[4]::]
                q1_lm_2 = q1[new_slam_index[4]::]
                q2_lm_2 = q2[new_slam_index[4]::]
                q3_lm_2 = q3[new_slam_index[4]::]
                q4_lm_2 = q4[new_slam_index[4]::]
                label_lm_2 = sc0_string + sc2_string

                txt_data.close()

                # convert txt data to a class according to CovertRefFrame Class
                lm_0 = ConvertRefFrameLocalizationOnly(f_loc, time_lm_0, label_lm_0, "C{1}-")
                lm_0.process_data(time_lm_0, x_lm_0, y_lm_0, z_lm_0, q1_lm_0, q2_lm_0, q3_lm_0, q4_lm_0)

                lm_1 = ConvertRefFrameLocalizationOnly(f_loc, time_lm_1, label_lm_1, "C{2}--")
                lm_1.process_data(time_lm_1, x_lm_1, y_lm_1, z_lm_1, q1_lm_1, q2_lm_1, q3_lm_1, q4_lm_1)

                lm_2 = ConvertRefFrameLocalizationOnly(f_loc, time_lm_2, label_lm_2, "C{3}--")
                lm_2.process_data(time_lm_2, x_lm_2, y_lm_2, z_lm_2, q1_lm_2, q2_lm_2, q3_lm_2, q4_lm_2)

                # LM_0.process_data(time, x, y, z, q1, q2, q3, q4)

                # next try to convert them to json files into the correct directory.
                # save the json file in the directory of the utilized rosbag for localization only mode.
                # the file name should be LM_MapName_nr
                SaveJsonInCorrectDir(town, SL, lm_0, sc0_string, sc0_string, file_nr)
                SaveJsonInCorrectDir(town, SL, lm_1, sc0_string, sc1_string, file_nr)
                SaveJsonInCorrectDir(town, SL, lm_2, sc0_string, sc2_string, file_nr)
                print("Converted {}".format(file))


def SaveJsonInCorrectDir(town, SL, crf, map_string, loc_string, file_nr):
    # saves the crf object to the correct directory in a json file
    if loc_string == "Stat":
        # In which directory should the json file be saved
        json_dir_save = base_dir + "/stuckbehindvan/20fps/T{}_SL{}_s".format(town, SL)

    elif loc_string == "Sc1":
        json_dir_save = base_dir + "/stuckbehindvan/20fps/T{}_SL{}_d15".format(town, SL)

    elif loc_string == "Sc2":
        json_dir_save = base_dir + "/VansOppositeRoad/T{}_SL{}_d10".format(town, SL)

    else:
        print("wrong naming convention txt file")
        return

    json_file_name = "/T{}_SL{}_LM_{}{}_{}".format(town, SL, map_string, loc_string, file_nr)

    crf2json(crf, json_dir_save, json_file_name)

    # print("One file converted to json, look in directory: {}".format(json_dir_save))


class ConvertRefFrameLocalizationOnly:
    def __init__(self, flocation, time, label, plotstyle):
        self.flocation = flocation

        self.plotstyle = plotstyle

        self.label = label

        self.time = time

        self.positions = []

        self.orientations = []

        # orientation expressed in quaternions
        self.quaternions = []

        # Homogeneous coordinate matrix
        self.Q = []

        # Contains the data used for the open() function
        self.data = {}

        # Relative pose change over a certain time frame expressed in homogeneous coordinates
        self.Q1Q2 = []

        # time stamps used for RPE
        self.timeQ1Q2 = []

        # localization mode
        self.method = "LM"

    def process_data(self, time, x, y, z, q1, q2, q3, q4):
        # convert to the appropriate reference system

        # need to extract the Euler angles to convert yaw since it only indicates angle at +180, -180
        roll_list = []
        pitch_list = []
        yaw_180_list = []

        for i, t in enumerate(time):
            x_old = x[i]
            y_old = y[i]
            z_old = z[i]
            q1_old = q1[i]
            q2_old = q2[i]
            q3_old = q3[i]
            q4_old = q4[i]

            # Convert orb axis to ros axis, which in theory looks like
            # 1) CCW (+)90 deg in x-axis
            # 2) CCW (+)90 deg in z-axis

            # But it seems that positive rotations are defined as CW in the ORB system, instead of CCW.
            # That is the only reason how I can explain that x is correct and y and z are in the negative direction.
            # It is also not a left handed system, because the orientation of the axis are right handed.
            theta = np.radians(-90)
            c, s = np.cos(theta), np.sin(theta)
            # Right handed rotation matrices
            Rx = np.matrix([[1, 0, 0], [0, c, -s], [0, s, c]])
            Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])

            pos_old = np.matrix([x_old, y_old, z_old])
            new_pos = Rz * Rx * pos_old.reshape(3, 1)
            x_new = new_pos.tolist()[0][0]
            y_new = new_pos.tolist()[1][0]
            z_new = new_pos.tolist()[2][0]

            position = np.array([x_new, y_new, z_new])
            self.positions.append(position)

            # Quaternions is defined as iq1+jq2+kq3+q4
            # The axis system of i, j, k  are incorrect and need to be converted to ROS axis system
            q_old = np.matrix([q1_old, q2_old, q3_old])
            q_new = Rz * Rx * q_old.reshape(3, 1)
            # Now the quaternion axis system has the same orientation as the euclidean space,
            # since the ORB axis system underwent the same transformation
            q1_new = q_new.tolist()[0][0]
            q2_new = q_new.tolist()[1][0]
            q3_new = q_new.tolist()[2][0]
            q4_new = q4_old# localization mode

            quaternion = [q1_new, q2_new, q3_new, q4_new]
            self.quaternions.append(quaternion)
            # q = tf.transformations.quaternion_matrix(quaternion)
            q = transformations.quaternion_matrix(quaternion)
            q[0][3] = x_new
            q[1][3] = y_new
            q[2][3] = z_new
            self.Q.append(q)

            # Note that euler_from_quaternion, is quaternion_matrix function and then euler_from_matrix function
            # Also, positive rotations seem to be defined as CW instead of CCW.
            # roll, pitch, yaw_180 = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
            roll, pitch, yaw_180 = transformations.euler_from_quaternion(quaternion, axes='sxyz')

            # roll and pitch are exactly reversed. If results are weird, this could be the mistake
            roll_list.append(np.degrees(-roll))
            pitch_list.append(np.degrees(-pitch))
            yaw_180_list.append(np.degrees(yaw_180))

        # convert yaw angle to an angle that can show more than 180 degrees
        yaw_list = self.abs_yaw_angle_conversion(yaw_180_list)

        # append the converted euler angles into the orientation attribute
        for index, roll in enumerate(roll_list):
            orientation = np.array([roll, pitch_list[index], yaw_list[index]])
            self.orientations.append(orientation)

    def abs_yaw_angle_conversion(self, rel_yaw_angle):
        """Function that converts a yaw angle that ranges from  -180 to 180 degrees, to a yaw angle that has infinite range
        and indicates full rotations. Note that this function can be used for left and right handed axis system"""

        yaw_abs = []
        yaw_abs.append(rel_yaw_angle[0])

        # n180 tracks the number of 180 degrees rotations and direction.
        n180 = 0
        n360 = 0

        # sign_modulo is the sign of the very first pose
        # scratch that... second pose, first pose the angle can be zero for ORB SLAM. The method does not work then.
        sign_modulo = np.sign(rel_yaw_angle[1])
        index = 1
        index_limit = len(rel_yaw_angle)

        while index != index_limit:
            # finds the modulo of either 180 or -180
            modulo_angle = rel_yaw_angle[index] % (sign_modulo*180)

            if myround(rel_yaw_angle[index - 1]) == -180 and myround(rel_yaw_angle[index]) == 180:
                # for some weird reason this newer UE can output an angle of 180.035
                if rel_yaw_angle[index] > 180:
                    n180 = n180 + 180
                n180 = n180 - 180
                # print(0, index)
                # pdb.set_trace()

            if myround(rel_yaw_angle[index - 1]) == 180 and myround(rel_yaw_angle[index]) == -180:
                # print(1, index)
                if rel_yaw_angle[index-1] > 180:
                    n180 = n180 - 180
                n180 = n180 + 180

            if myround(rel_yaw_angle[index]) == 0 and np.sign(rel_yaw_angle[index - 1]) == -1 and np.sign(rel_yaw_angle[index]) == 1:
                # print(2, index)
                n180 = n180 + 180

            if myround(rel_yaw_angle[index]) == 0 and np.sign(rel_yaw_angle[index - 1]) == 1 and np.sign(rel_yaw_angle[index]) == -1:
                # print(3, index)
                n180 = n180 - 180


            yaw_abs_element = n180 + modulo_angle
            yaw_abs.append(yaw_abs_element)
            index = index + 1

        return yaw_abs


if __name__ == "__main__":
    main()
