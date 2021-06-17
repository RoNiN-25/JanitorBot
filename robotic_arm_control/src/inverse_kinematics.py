#! /usr/bin/env python
# import required libraries
from math import sin, cos, atan2, sqrt, fabs
import rospy

# function for inverse kinematics
def iKine(point):
    x = point[0]
    y = point[1]
    z = point[2]

    theta = atan2(-x,y)
    vert_dist = z + 0.535
    hor_dist = 1.4 - sqrt((x**2 + y**2))

    return [theta, vert_dist, hor_dist]
