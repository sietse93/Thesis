"""
Allow easy labeling of scenarios including: behind moving vehicle, behind stopping vehicle, moving vehicle other lane,
stopping vehicle other lane. This code should assist in understanding the data from the CARLA experiment

"""
import numpy as np


class ScenarioProcessor:
    def __init__(self, flocation, SL, NV):
        self.flocation_da = flocation + "SL_{}_NV_{}_SV_1_da.txt".format(SL, NV)
        self.raw_data_da = {}
        self.NV = NV

        self.flocation_hero = flocation + "SL_{}_NV_{}_SV_1_gt.txt".format(SL, NV)

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
            first_line_data = data_single_vehicle[0].split(" ")
            initiate_vehicle = Vehicle(int(first_line_data[1]), [[float(first_line_data[0])*10**(-3),
                                                                 float(first_line_data[2]),
                                                                 float(first_line_data[3])]])
            # append rest of the data to the vehicle class
            rest_data_single_vehicle = data_single_vehicle[1:]
            for raw_line_data in rest_data_single_vehicle:
                line_data = raw_line_data.split(" ")
                time_stamp = float(line_data[0])*10**(-3)
                x = float(line_data[2])
                y = float(line_data[3])
                initiate_vehicle.data.append([time_stamp, x, y])

            vehicles.append(initiate_vehicle)

        return vehicles

    def process_hero(self):
        """Put the gt txt data into a vehicle class"""
        data = self.raw_data_gt.readlines()
        first_line_data = data[0].split(" ")
        hero = Vehicle(0, [[float(first_line_data[0])*10**(-3),
                            float(first_line_data[1]),
                            float(first_line_data[2])]])
        rest_raw_data = data[1:]
        for line_raw in rest_raw_data:
            line_data = line_raw.split(" ")
            time_stamp = float(line_data[0]) * 10 ** (-3)
            x = float(line_data[1])
            y = float(line_data[2])
            hero.data.append([time_stamp, x, y])

        return hero

    def encountered_vehicles_filter(self, hero, dynamic_agents):
        """Provide a list of vehicles that the hero encountered"""

        encountered_vehicles = []
        id_encountered_vehicles = []

        for index, data_line in enumerate(hero.data[:-1]):

            hero_x = data_line[1]
            hero_y = data_line[2]

            vis_range = 20  # range threshold in which a dynamic agent can be seen longitudinal
            side_range = 5  # range threshold in which a dynamic agent can be seen lateral
            # try:
            # What to do with the when the vehicle stands still (this also happens in the beginning)
            # find out which direction the hero is travelling
            xdirection = np.sign(hero.data[index+1][1] - hero_x)
            ydirection = np.sign(hero.data[index+1][2] - hero_y)

            # find out which vehicles the hero encounters at each timestamp
            for vehicle in dynamic_agents:
                encountered = False
                vehicle_time = vehicle.data[index][0]
                vehicle_x = vehicle.data[index][1]
                vehicle_y = vehicle.data[index][2]

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


class Vehicle:
    def __init__(self, identity, line_data):
        # contains unique id of the vehicle
        self.id = identity
        # contains the data of this vehicle, a list of [timestamp, x, y ] (seconds and world coordinates)
        self.data = line_data


class EncounteredVehicle(Vehicle):
    def __init__(self, vehicle):
        # All the vehicle data
        Vehicle.__init__(self, vehicle.id, vehicle.data)

        # Only the data where the vehicle is encountered
        self.encounter_data = []
        self.begin_time = ()
        self.end_time = ()



def main():
    flocation = "/home/sietse/official_experiment_data/"
    with ScenarioProcessor(flocation=flocation, SL=40, NV=40) as SP:
        dynamic_agents = SP.process_dynamic_agents()
        hero = SP.process_hero()
        encountered_vehicles = SP.encountered_vehicles_filter(hero, dynamic_agents)


if __name__ == "__main__":
    main()
