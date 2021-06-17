#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Float64
from math import sin, cos, atan2, sqrt, fabs

rospy.init_node('joint_positions_node', anonymous=True)
rate = rospy.Rate(80)
# Define publishers for joint1 and joint2 position controller commands.
twist_joint_pub = rospy.Publisher('/janitor/twist_joint_position_controller/command', Float64, queue_size=10)
pris_gripper_pub = rospy.Publisher('/janitor/prismatic_gripper_position_controller/command', Float64, queue_size=10)
pris_vertical_pub = rospy.Publisher('/janitor/prismatic_vertical_position_controller/command', Float64, queue_size=10)
pris_horizontal_pub = rospy.Publisher('/janitor/prismatic_horizontal_position_controller/command', Float64, queue_size=10)
while(True):
    twist_joint_position = -1.57
    pris_gripper_position = -0.25
    pris_vertical_position = 0.5
    pris_horizontal_position = 0.4

    twist_joint_pub.publish(twist_joint_position)
    pris_gripper_pub.publish(pris_gripper_position)
    pris_vertical_pub.publish(pris_vertical_position)
    pris_horizontal_pub.publish(pris_horizontal_position)
