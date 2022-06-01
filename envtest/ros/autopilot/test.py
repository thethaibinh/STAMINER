import numpy as np
from numpy import pi 
# print(numpy.rad2deg(2 * numpy.pi))
# print((pi / 4 + 4 * pi) % (2 * pi) * 180 / pi)

# def constrainAngle(x):
#     x = np.fmod(x + pi, pi * 2)
#     if (x < 0):
#         x += 2*pi
#     return x - pi

def wrap_360_cd(angle):
    res = np.fmod(angle, 36000.0)
    if (res < 0):
        res += 36000.0
    return res

def wrap_180_cd(angle):
    res = wrap_360_cd(angle)
    if (res > 18000.0): 
        res -= 36000.0
    return res

def wrap_360_rad(angle):
    res = np.fmod(angle, 2*pi)
    if (res < 0):
        res += 2*pi
    return res

def wrap_180_rad(angle):
    res = wrap_360_rad(angle)
    if (res > pi): 
        res -= 2*pi
    return res

# print(wrap_180_rad(359 / 180 * pi) * 180 / pi)
# print(constrainAngle(270 / 180 * pi) * 180 / pi)