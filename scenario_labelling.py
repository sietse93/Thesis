"""
Allow easy labeling of scenarios including: behind moving vehicle, behind stopping vehicle, moving vehicle other lane,
stopping vehicle other lane. This code should assist in understanding the data from the CARLA experiment

"""
import numpy as np
import math
import string


class ScenarioProcessor:
    def __init__(self, flocation, SL, NV):
        self.flocation_da = flocation + "SL_{}_NV_{}_SV_1_da.txt".format(SL, NV)
        self.raw_data_da = {}
        self.NV = NV

        self.flocation_hero = flocation + "SL_{}_NV_{}_SV_1_gt.txt".format(SL, NV)

        self.fps =40

    def __enter__(self):
        # data consists of timestamp, vehicle id, x and y location
        self.raw_data_da = open(self.flocation_da, 'r')
        self.raw_data_gt = open(self.flocation_hero, 'r')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.raw_data_da.close()
        self.raw_data_gt.close()
        return None

    def process_dynamic_agents(self):
        """Sort dynamic agent data into a list vehicles in the world"""
        # this will be a list of class Vehicles
        vehicles = []

        # read all data
        data = self.raw_data_da.readlines()
        nv = self.NV

        # the data is stored in game_timestamp, id, x, y.
        # If the total number of vehicles is 40, that means that 40 lines later the same vehicle data is stored
        for i in range(nv):
            # this contains all data from a single vehicle
            data_single_vehicle = data[i::nv]

            # create the vehicle class
            # first_line_data = data_single_vehicle[0].split(" ")
            # lets test what the data does after the vehicles are spawned
            # only use the data after 1 second of simulation which is equal to the nr fps
            first_line_data = data_single_vehicle[self.fps].split(" ")
            initiate_vehicle = Vehicle(int(first_line_data[1]), [[round(float(first_line_data[0])*10**(-3), 3),
                                                                 round(float(first_line_data[2]), 2),
                                                                 round(float(first_line_data[3]), 2)]])
            initiate_vehicle.time.append(round(float(first_line_data[0])*10**(-3), 3))
            # append rest of the data to the vehicle class
            # rest_data_single_vehicle = data_single_vehicle[1:]
            rest_data_single_vehicle = data_single_vehicle[self.fps+1:]
            for raw_line_data in rest_data_single_vehicle:
                line_data = raw_line_data.split(" ")
                time_stamp = round(float(line_data[0])*10**(-3), 3)
                x = round(float(line_data[2]), 2)
                y = round(float(line_data[3]), 2)
                initiate_vehicle.data.append([time_stamp, x, y])
                initiate_vehicle.time.append(time_stamp)

            vehicles.append(initiate_vehicle)

        return vehicles

    def process_hero(self):
        """Put the gt txt data into a vehicle class"""
        data = self.raw_data_gt.readlines()
        # first_line_data = data[0].split(" ")
        first_line_data = data[self.fps].split(" ")
        hero = HeroVehicle(0, [[round(float(first_line_data[0])*10**(-3), 3),
                                round(float(first_line_data[1]), 2),
                                round(float(first_line_data[2]), 2)]])
        hero.heading.append(round(float(first_line_data[6]), 0))
        # rest_raw_data = data[1:]
        rest_raw_data = data[self.fps+1:]
        for line_raw in rest_raw_data:
            line_data = line_raw.split(" ")
            time_stamp = round(float(line_data[0]) * 10 ** (-3), 3)
            x = round(float(line_data[1]), 2)
            y = round(float(line_data[2]), 2)
            hero.heading.append(round(float(line_data[6]), 0))
            hero.data.append([time_stamp, x, y])

        return hero

    def encountered_vehicles_filter(self, hero, dynamic_agents):
        """Provide a list of vehicles that the hero encountered"""

        encountered_vehicles = []
        id_encountered_vehicles = []

        # you want the data after that the vehicle is spawned
        for index, data_line in enumerate(hero.data[:-1]):
            hero_time = data_line[0]
            hero_x = data_line[1]
            hero_y = data_line[2]
            hero_head_deg = hero.heading[index]
            hero_head = math.radians(hero_head_deg)

            vis_range = 20  # range threshold in which a dynamic agent can be seen longitudinal
            side_range = 15  # range threshold in which a dynamic agent can be seen lateral
            # try:
            # What to do with the when the vehicle stands still (this also happens in the beginning)
            # find out which direction the hero is travelling
            disp_x = hero.data[index+1][1] - hero_x
            disp_y = hero.data[index+1][2] - hero_y
            xdirection = np.sign(disp_x)
            ydirection = np.sign(disp_y)

            # find out which vehicles the hero encounters at each timestamp
            for vehicle in dynamic_agents:
                encountered = False
                # get the timestamps from the vehicle
                # find the equivalent timestamp
                eq_index = vehicle.time.index(hero_time)
                vehicle_time = vehicle.time[eq_index]
                vehicle_x = vehicle.data[eq_index][1]
                vehicle_y = vehicle.data[eq_index][2]
                if xdirection == 0 and ydirection == 0:
                    if abs(hero_head_deg) == 0:
                        xdirection = 1.0
                    if hero_head_deg == 90:
                        ydirection = 1.0
                    if hero_head_deg == -90:
                        ydirection = -1.0
                    if 175 < abs(hero_head_deg) <= 180:
                        xdirection = -1.0

                # check if vehicle within range in the x-direction
                if xdirection == 1.0:
                    # vehicle should be within longitudinal range, but also lateral
                    if hero_x <= vehicle_x <= (hero_x+vis_range) and \
                            (hero_y-side_range) < vehicle_y <= (hero_y+side_range):
                        encountered = True
                elif xdirection == -1.0:
                    if (hero_x-vis_range) <= vehicle_x <= hero_x and \
                            (hero_y - side_range) < vehicle_y <= (hero_y + side_range):
                        encountered = True

                # check if vehicle within range in y-direction
                if ydirection == 1.0:
                    if hero_y <= vehicle_y <= (hero_y+vis_range) and \
                            (hero_x-side_range) < vehicle_x <= (hero_x+side_range):
                        encountered = True

                elif ydirection == -1.0:
                    if (hero_y-vis_range) <= vehicle_y <= hero_y and \
                            (hero_x - side_range) < vehicle_x <= (hero_x + side_range):
                        encountered = True

                # save the encountered vehicles in a list
                if encountered:
                    if vehicle.id not in id_encountered_vehicles:
                        # if it is a new vehicle, save its id and make a new EncounteredVehicleObject
                        id_encountered_vehicles.append(vehicle.id)
                        new_encountered_vehicle = EncounteredVehicle(vehicle)
                        new_encountered_vehicle.begin_time = vehicle_time
                        new_encountered_vehicle.encounter_data.append([vehicle_time, vehicle_x, vehicle_y])
                        encountered_vehicles.append(new_encountered_vehicle)
                    else:
                        # if vehicle is already encountered add the location and timestamp.
                        index = id_encountered_vehicles.index(vehicle.id)
                        encountered_vehicles[index].encounter_data.append([vehicle_time, vehicle_x, vehicle_y])

        # add the end time of the encountered vehicles in the class
        for encountered_vehicle in encountered_vehicles:
            final_encounter_time = encountered_vehicle.encounter_data[-1][0]
            encountered_vehicle.end_time = final_encounter_time

        return encountered_vehicles

    # to do: scenario creator does not work!!!
    def scenario_creator(self, hero, encountered_vehicles):
        """Converts location data into Scenario objects"""

        hero_time = [data_line[0] for data_line in hero.data]
        for encountered_vehicle in encountered_vehicles:

            # extract encountered vehicle time
            vehicle_time = [data_line[0] for data_line in encountered_vehicle.encounter_data]

            # find the equivalent hero indexed time
            begin_time = encountered_vehicle.begin_time
            end_time = encountered_vehicle.end_time
            begin_index = hero_time.index(begin_time)
            end_index = hero_time.index(end_time)

            # this describes the location of the hero when encountered vehicle met the hero
            # hero_encountered = hero.data[begin_index:end_index]
            # hero_encountered_time = [data_line[0] for data_line in hero_encountered]

            # for every time stamp of encountered vehicle, we want a string that describes what happened
            # loop through every time stamp from encountered vehicle,
            # match it with the hero time and
            # check if it was standing still
            temp_scenarios_list = []
            for index, time in enumerate(vehicle_time[5:]):
                # equivalent hero index
                hero_index = hero_time.index(time)
                # check if hero is going to move
                hero_disp_x = hero.data[hero_index][1] - hero.data[hero_index-5][1]
                hero_disp_y = hero.data[hero_index][1] - hero.data[hero_index-5][1]
                if hero_disp_x == 0 and hero_disp_y == 0:
                    hero_string = "hero static"
                else:
                    hero_string = "hero dynamic"
                # check if vehicle is moving
                # vehicle_disp_x = encountered_vehicle.encounter_data[index][1] - encountered_vehicle.encounter_data[index-5][1]
                # vehicle_disp_y = encountered_vehicle.encounter_data[index][2] - encountered_vehicle.encounter_data[index-5][2]
                # if vehicle_disp_x == 0 and vehicle_disp_y == 0:
                #     vehicle_string = "agent static"
                # else:
                #     vehicle_string = "agent dynamic"

                # add other things you want to describe here

                scenario_string = hero_string # + ", " + vehicle_string
                temp_scenarios_list.append(scenario_string)

            #everything up to here works

            # put everything in a Scenario Class, so begin and end time can be plotted
            # check every scenario for the vehicle, if a new scenario occurs, begin and end time should be in a
            # scenario class and should be appended to the scenario_list property of the vehicle.
            for index, scenario_description in enumerate(temp_scenarios_list):
                scenario_time = vehicle_time[index]
                if index == 0:
                    old_scenario = Scenario(scenario_description, scenario_time)
                    old_scenario.timestamps.append([scenario_time])
                else:
                    if scenario_description == old_scenario.description:
                        old_scenario.timestamps.append([scenario_time])
                    else:
                        # create a new scenario
                        new_scenario = Scenario(scenario_description, scenario_time)
                        print("new scenario description: {}".format(new_scenario.description))
                        print("old scenario description: {}".format(old_scenario.description))
                        print(scenario_time)
                        # make sure the old scenario has all its properties
                        old_scenario.end_time = scenario_time
                        print(old_scenario.timestamps)
                        # add the the encountered_vehicles properties
                        encountered_vehicle.scenario_list.append([old_scenario])

                        # forget the old scenario
                        old_scenario = new_scenario
        return encountered_vehicles

    def vehicle_labeling(self, list_encountered_vehicles):
        alphabet = list(string.ascii_letters)
        color_code = 0
        for index, vehicle in enumerate(list_encountered_vehicles):
            vehicle.label = alphabet[index]
            vehicle.marker = "${}$".format(alphabet[index])
            vehicle.color = "C{}".format(color_code)
            if color_code == 9:
                color_code = 0
            else:
                color_code = color_code + 1
        return list_encountered_vehicles


class Vehicle:
    """"A vehicle spawned in the CARLA"""
    def __init__(self, identity, line_data):
        # contains unique id of the vehicle
        self.id = identity
        # contains the data of this vehicle, a list of [timestamp, x, y ] (seconds and world coordinates)
        self.data = line_data
        self.time = []


class HeroVehicle(Vehicle):
    """Our Hero, now with heading"""
    def __init__(self, identity, line_data):
        Vehicle.__init__(self, identity, line_data)
        self.heading = []


class EncounteredVehicle(Vehicle):
    """"A Vehicle encountered by hero"""
    def __init__(self, vehicle):
        # All the vehicle data
        Vehicle.__init__(self, vehicle.id, vehicle.data)

        # Only the data where the vehicle is encountered
        self.encounter_data = []
        self.begin_time = ()
        self.end_time = ()

        # temporary scenario list
        self.scenario_list = []

        # plot characteristics
        self.label = ""
        self.marker = ""
        self.color = ''


class Scenario:
    """Description of the situation when the vehicle was encountered"""
    def __init__(self, description, start_time):
        self.description = description
        self.timestamps = []
        self.begin_time = start_time
        self.end_time = ()


def main():
    flocation = "/home/sietse/official_experiment_data/"
    with ScenarioProcessor(flocation=flocation, SL=40, NV=40) as SP:
        dynamic_agents = SP.process_dynamic_agents()
        hero = SP.process_hero()
        encountered_vehicles = SP.encountered_vehicles_filter(hero, dynamic_agents)
        SP.vehicle_labeling(encountered_vehicles)

        # plot_data = []
        for encountered_vehicle in encountered_vehicles:
            print(encountered_vehicle.label)
        #     plot_data.append([encountered_vehicle.id, encountered_vehicle.begin_time, encountered_vehicle.end_time])
        # print(plot_data)
        # encountered_vehicle = SP.scenario_creator(hero, encountered_vehicles[0:3])


if __name__ == "__main__":
    main()
