from matplotlib import pyplot as plt
import numpy as np
import pdb


def convert_gt(rel_yaw_angle):
    yaw_abs = []
    sign_modulo = np.sign(rel_yaw_angle[1])
    index = 1
    index_limit = len(rel_yaw_angle)
    n360 = 0
    yaw_abs.append(rel_yaw_angle[0])
    while index != index_limit:
        modulo_angle = rel_yaw_angle[index] % (sign_modulo*360)
        if myround(rel_yaw_angle[index]) == 0 and myround(rel_yaw_angle[index-1]) == 0 and np.sign(rel_yaw_angle[index - 1]) == -1 and np.sign(
                rel_yaw_angle[index]) == 1:
            n360 = n360 + 360
        if myround(rel_yaw_angle[index]) == 0 and myround(rel_yaw_angle[index-1]) == 0 and np.sign(rel_yaw_angle[index - 1]) == 1 and np.sign(
                rel_yaw_angle[index]) == -1:
            n360 = n360 - 360
        if myround(rel_yaw_angle[index]) == 360 and myround(rel_yaw_angle[index-1]) == 0 and np.sign(rel_yaw_angle[index-1]) == 1:
            n360 = n360 - 360
            #pdb.set_trace()
        if myround(rel_yaw_angle[index]) == 0 and myround(rel_yaw_angle[index - 1]) == 360 and np.sign(rel_yaw_angle[index - 1]) == 1:
            n360 = n360 + 360
        yaw_abs_element = n360 + modulo_angle
        yaw_abs.append(yaw_abs_element)
        index = index + 1
    #pdb.set_trace()
    return yaw_abs


def myround(x, base=10):
    """rounds the number to the nearest base, in this case 10
 ORB has jumps that go from -175 to 175, which are not detected if the round() function is used"""
    return int(round(x/base))*base

def main():
    Town = 3
    SL = 132
    base_dir = "/home/sietse/results_carla0.9/stuckbehindvan/20fps/"

    dir = "T{}_SL{}_s/".format(Town, SL)
    file = "T{}_SL{}_s_gt.txt".format(Town, SL)

    file_loc = base_dir + dir + file
    data = open(file_loc, 'r')

    time = []
    rel_yaw_angle = []

    for data_line in data:
        line = data_line.split(" ")
        float_line = [float(element) for element in line]
        time.append(float_line[0])
        rel_yaw_angle.append(float_line[6])

    yaw = convert_gt(rel_yaw_angle)

    plt.plot(time, rel_yaw_angle)
    plt.plot(time, yaw)

    plt.show()

if __name__=="__main__":
    main()
