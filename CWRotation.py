from sympy import *
import numpy as np

c_a = Symbol('c(alpha)')
s_a = Symbol('s(alpha)')
c_b = Symbol('c(beta)')
s_b = Symbol('s(beta)')
c_g = Symbol('c(gamma)')
s_g = Symbol('s(gamma)')

Rx = np.matrix([[1, 0, 0], [0, c_a, s_a], [0, -s_a, c_a]])
Ry = np.matrix([[s_b, 0, c_b], [0, 1, 0], [c_b, 0, -s_b]])
Rz = np.matrix([[c_g, s_g, 0], [-s_g, c_g, 0], [0, 0, 1]])

Rot_matrix = Rz*Ry*Rx
Rot_list = Rot_matrix.tolist()
print(Rot_list)
R11 = Rot_list[0][0]
R12 = Rot_list[0][1]
R13 = Rot_list[0][2]
R21 = Rot_list[1][0]
R22 = Rot_list[1][1]
R23 = Rot_list[1][2]
R31 = Rot_list[2][0]
R32 = Rot_list[2][1]
R33 = Rot_list[2][2]

# R31 -> acos('beta')
# acos('beta')

# R32/R33 = s(alpha)*s(beta)/-c(alpha)*s(beta) = -atan2(R32, R33)

# R21/R11 = -s(beta)*s(gamma)/c(gamma)*s(beta)
#-atan(R21,R11)
print(R21, R11)