"""
Allow easy labeling of scenarios including: behind moving vehicle, behind stopping vehicle, moving vehicle other lane,
stopping vehicle other lane. This code should assist in understanding the data from the CARLA experiment

"""


class ScenarioProcessor:
    def __init__(self, flocation, SL, NV):
        self.flocation_da = flocation + "SL_{}_NV_{}_SV_1_da.txt".format(SL, NV)
        self.raw_data = {}
        self.NV = NV

    def __enter__(self):
        # data consists of timestamp, vehicle id, x and y location
        self.raw_data = open(self.flocation_da, 'r')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.raw_data.close()
        return None

    def process_data(self):
        """Sort data into a list vehicles"""
        # this will be a list of class Vehicles
        vehicles = []

        # read all data
        data = self.raw_data.readlines()
        nv = self.NV

        # the data is stored in game_timestamp, id, x, y.
        # If the total number of vehicles is 40, that means that 40 lines later the same vehicle data is stored
        for i in range(nv):
            # this contains all data from a single vehicle
            data_single_vehicle = data[i:: i + nv]

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

class Vehicle:
    def __init__(self, identity, line_data):
        # contains unique id of the vehicle
        self.id = identity
        # contains the data of this vehicle, a list of [timestamp, x, y ] (seconds and world coordinates)
        self.data = line_data


def main():
    flocation = "/home/sietse/official_experiment_data/"
    with ScenarioProcessor(flocation=flocation, SL=40, NV=40) as DA:
        DA.process_data()


if __name__ == "__main__":
    main()
