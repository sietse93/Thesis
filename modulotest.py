from matplotlib import pyplot as plt
import numpy as np

data = open("/home/sietse/carla_experiment_data/dynamic_loopclosed_gt.txt", "r")

time = []
x = []
y = []
z = []
roll = []
pitch = []
yaw = []

for line in data:
    split_line = line.split(" ")
    float_line = [float(element) for element in split_line]
    time.append(round(float_line[0] * 10 ** (-3), 2))
    x.append(float_line[1])
    y.append(float_line[2])
    z.append(float_line[3])
    roll.append(float_line[4])
    pitch.append(float_line[5])
    yaw.append(float_line[6])

yaw_filter = []

for index, element in enumerate(yaw):
    if index == 0:
        yaw_filter.append(element)
    elif index == (len(yaw)-1):
        yaw_filter.append(element)
    # if the yaw angle after this time stamp and before this time stamp has a different sign
    # flip the sign
    elif np.sign(element) != np.sign(yaw_filter[index - 1]) and np.sign(element) != np.sign(yaw[index + 1]):
        yaw_filter.append(-element)
    else:
        yaw_filter.append(element)

# This is some if loop I thought of that is too complicated but might need revisiting if the other concept does not work
# if abs(yaw_angle) > 178 and np.sign(yaw_angle) != np.sign(yaw_filter[index-1] and np.sign != np.sign(yaw_filter[index -2])):

yaw_abs = []
yaw_abs.append(yaw_filter[0])
yaw_abs.append(yaw_filter[1])
index = 2

# there is noise going on at the end of this run. First focus on getting the algorithm going and then see how it handles noise
# that is why the index is going to 8000
index_limit = 6000



while index != index_limit:

    # If there is a jump in the data, we need to do some special calculations to get the absolute yaw angle
    if abs(yaw_filter[index]-yaw_filter[index-1]) > 355:

        # first180 is the value before the yaw angle flips sign
        first180 = yaw_filter[index - 1]



        # as long as the looped angle does not have the same sign as first180, this modulo calculation will work.
        while np.sign(yaw_filter[index]) != np.sign(first180) and index != index_limit:
            relative_angle_from_first180 = yaw_filter[index] % round(first180)
            yaw_abs_element = first180 + relative_angle_from_first180
            yaw_abs.append(yaw_abs_element)
            index = index + 1

        # if the looped angle does have the same sign as first180, 1 of 2 things could have happened
        # the vehicle has moved back towards the first180 yaw position and has a value of around 180 degrees
        # this means that the actual value is again the absolute value
        if np.sign(yaw_filter[index]) == np.sign(first180) and round(yaw_filter[index]) == 180:
            yaw_abs_element = yaw_filter[index]
            yaw_abs.append(yaw_abs_element)
            index = index + 1

            # I think I can put a break here and it will have the same effect

        # the vehicle keeps turning towards the same direction and has a value of around 0 degrees
        # this means that the absolute angle is now 2*180 degrees
        if np.sign(yaw_filter[index]) == np.sign(first180) and round(yaw_filter[index]) == 0:
            # redefine the absolute angle from where is measured, which is the last angle of absolute yaw
            # this means it still has the same sign
            first360 = yaw_abs[-1]
            while np.sign(yaw_filter[index]) == np.sign(first360) and index != index_limit:
                # Modulo can be used here as well, this would allow for some recursive function
                yaw_abs_element = first360 + yaw_filter[index]
                yaw_abs.append(yaw_abs_element)
                print(yaw_abs_element)
                print(yaw_filter[index])
                print(index)
                index = index + 1

    # if there has been no jump in the data, the angle given is the absolute angle
    else:
        yaw_abs_element = yaw_filter[index]
        yaw_abs.append(yaw_abs_element)
        index = index + 1

time_limit = time[0:index_limit]

# So now the data is filtered. Note that there are still fluctuations.
# plt.figure()
# plt.subplot(3, 1, 1)
# plt.plot(time, x)
#
# plt.subplot(3, 1, 2)
# plt.plot(time, y)
#
# plt.subplot(3, 1, 3)
# plt.plot(time, yaw_filter)
#
# plt.figure()
# plt.plot(x, y, label="trajectory")
# plt.plot(x[0], y[0], 'x', label="starting point")
# plt.legend()

plt.figure()
plt.plot(time_limit, yaw_abs)

plt.show()

data.close()
