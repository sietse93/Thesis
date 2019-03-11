from matplotlib import pyplot as plt

def plot_vehicles_encountered(vehicles_encountered, loc_marker=1):
    for vehicle in vehicles_encountered:
        msize = 15
        plt.axvline(vehicle.begin_time, color=vehicle.color)
        plt.axvline(vehicle.end_time, color=vehicle.color)
        plt.plot(vehicle.begin_time, loc_marker, marker=vehicle.marker, markersize=msize, color=vehicle.color)
        plt.plot(vehicle.end_time, loc_marker, marker=vehicle.marker,  markersize=msize, color=vehicle.color)
    return
