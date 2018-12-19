from matplotlib import pyplot as plt
import numpy as np
import math

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
index_limit = len(yaw_filter)-1
time_limit = time[0:index_limit]

# absolute 180 degrees
n180 = 0
sign_modulo = np.sign(yaw_abs[1])
while index != index_limit:

    modulo_angle = yaw_filter[index] % (sign_modulo*180)
    if round(yaw_filter[index-1]) == -180 and round(yaw_filter[index]) == 180:
        n180 = n180 - 180
        print("180 anticlockwise at {}".format(index))
        print(n180)

    if round(yaw_filter[index-1]) == 180 and round(yaw_filter[index]) == -180:
        n180 = n180 + 180
        print("180 clockwise at {}".format(index))
        print(n180)

    if round(yaw_filter[index]) == 0 and np.sign(yaw_filter[index -1]) == -1 and np.sign(yaw_filter[index]) == 1:
        n180 = n180 + 180
        print("0 clockwise at {}".format(index))
        print(n180)

    if round(yaw_filter[index]) == 0 and np.sign(yaw_filter[index -1]) == 1 and np.sign(yaw_filter[index]) == -1:
        n180 = n180 - 180
        print("0 anticlockwise at {}".format(index))
        print(n180)

    yaw_abs_element = n180 + modulo_angle
    print(yaw_filter[index])
    yaw_abs.append(yaw_abs_element)
    index = index + 1





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
