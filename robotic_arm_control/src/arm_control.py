#!/usr/bin/env python
# import required libraries
import rospy
from std_msgs.msg import Float64
from math import sin, cos, atan2, sqrt, fabs, isnan
from geometry_msgs.msg import Point
from time import sleep
from control_msgs.msg import JointControllerState

from inverse_kinematics import iKine

# initalize the point as nan
point = [float('nan'),float('nan'),float('nan')]
gripper_pos = 0.0
hor_pos = 0.0
vert_pos = 0.2
twist_angle = -1.57

# Function to send desired positions to the arm
def send_data_to_arm(theta, vert_dist, hor_dist, grip, gripOnly = False):
    # move end effector only
    if gripOnly:
        i = 0
        pris_gripper_pub.publish(grip)
        while abs(grip - gripper_pos) > 0.01:
            i += 1
            print(i)
            if i > 12:
                break # If desired position not attained after 1.2s, break
            rate.sleep()
    else:
        print('Up') # Vertical slider movement
        pris_vertical_pub.publish(0.8)
        while abs(0.8 - vert_pos) > 0.06:
            rate.sleep()

        print('Rotating') # Rotation
        twist_joint_pub.publish(theta)
        while abs(theta - twist_angle) > 0.001:
            rate.sleep()

        print('Horizontal') # Horizontal slider movement
        pris_horizontal_pub.publish(hor_dist)
        while abs(hor_dist - hor_pos) > 0.001:
            rate.sleep()

        print('Grip') # Gripper/End effector movement
        i = 0
        pris_gripper_pub.publish(grip)
        while abs(grip - gripper_pos) > 0.01:
            i += 1
            print(i)
            if i > 12:
                break
            rate.sleep()

        print('Up') # Vertical slider movement
        pris_vertical_pub.publish(vert_dist)
        while abs(vert_dist - vert_pos) > 0.05:
            rate.sleep()

# function to pick up the object
def pick_up_object():
    pris_gripper_pub.publish(-0.4)
    sleep(3)
    pris_vertical_pub.publish(0.5)
    sleep(2)

# function to place the object in the bin
def place_object():
    send_data_to_arm(-2.44, 0.7, 1.1, -0.4)
    sleep(3)
    send_data_to_arm(-2.44,0.7,1.1,-0.4)

# call back for object position
def pointCb(data):
    global point
    x = data.x
    y = data.y
    z = data.z

    point = [x,y,z]

# call back for end effector position
def gripperCb(data):
    global gripper_pos
    gripper_pos = data.process_value

# call back for twist angle
def twistCb(data):
    global twist_angle
    twist_angle = data.process_value

# call back for Vertical slider position
def vertCb(data):
    global vert_pos
    vert_pos = data.process_value

# call back for horizontal slider position
def horCb(data):
    global hor_pos
    hor_pos = data.process_value


rospy.init_node('arm_control', anonymous=True) # initalize node
rate = rospy.Rate(10)
# Define publishers for joint1 and joint position controller commands.
twist_joint_pub = rospy.Publisher('/janitor/twist_joint_position_controller/command', Float64, queue_size=10)
pris_gripper_pub = rospy.Publisher('/janitor/prismatic_gripper_position_controller/command', Float64, queue_size=10)
pris_vertical_pub = rospy.Publisher('/janitor/prismatic_vertical_position_controller/command', Float64, queue_size=10)
pris_horizontal_pub = rospy.Publisher('/janitor/prismatic_horizontal_position_controller/command', Float64, queue_size=10)

rospy.Subscriber('/object_point_for_arm', Point, pointCb) # subsciber for object location

# Subscribers for the joint positions
rospy.Subscriber('/janitor/twist_joint_position_controller/state', JointControllerState, twistCb)
rospy.Subscriber('/janitor/prismatic_gripper_position_controller/state', JointControllerState, gripperCb)
rospy.Subscriber('/janitor/prismatic_vertical_position_controller/state', JointControllerState, vertCb)
rospy.Subscriber('/janitor/prismatic_horizontal_position_controller/state', JointControllerState, horCb)

print('Starting.....')
while not rospy.is_shutdown(): # as long as rospy is running
    x = point[0]
    y = point[1]
    z = point[2]
    # if point is nan => No object detected
    if isnan(x) or isnan(y) or isnan(z):
        print('NAN')
    else:
        theta, vert_dist, hor_dist = iKine(point) # inverse kinematics to obtain the desired joint positions
        send_data_to_arm(theta, vert_dist, hor_dist, 0) # move to object
        send_data_to_arm(theta,vert_dist, hor_dist, -0.4, gripOnly=True) # grip object
        print('picking up')
        send_data_to_arm(theta, 0.8, hor_dist, -0.4) # pick up object
        print('Picked!!!!!')
        send_data_to_arm(2.44, 0.7, 1.1, -0.4) # move to bin
        send_data_to_arm(2.44, 0.7, 1.1, 0, gripOnly=True) # drop object
        print('Placed')
    rate.sleep()
