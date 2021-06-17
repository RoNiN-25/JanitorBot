#! /usr/bin/env python
# import Required libraries
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import sensor_msgs.point_cloud2
import tf2_ros
from pyquaternion import Quaternion

frame = None #Variable to store the frame of the camera
pick_point = Point()

#class for converting the image
class ImageConverter:
    def __init__(self): # Constructor
        self.bridge = CvBridge()
    def convertImage(self,data): # convert the image
        return self.bridge.imgmsg_to_cv2(data)


#function for pre-processing
def pre_process(image,lower_color,upper_color):

    #pre-processing for better detection
    blurred = cv2.GaussianBlur(image, (11, 11), 0)# blur
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)#Convert to HSV
    mask = cv2.inRange(hsv, lower_color, upper_color)    #Generate mask
    mask = cv2.erode(mask, None, iterations=1) #erode
    mask = cv2.dilate(mask, None, iterations=1) #dialte
    res = cv2.bitwise_and(image,image, mask= mask) # superimpose the mask on the image
    return res,mask

def find_contour(mask):      #For the contour of the blue box
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE) #function to get the contours

    # CHAIN_APPROX_NONE has been used since it gives different sized points for contours
    # Larger contours can will have larger number of points. So it has been used for obtain the largest contour
    # NOTE : Assumption made here is that, the biggest blue object in the frame will be the object itself

    k = 0 # variable for storing the index of the largest contour
    try:
        k = contours.index(max(contours, key = len))
    except:
        pass

    return k,contours

#to get the necessary points within the contour
def get_points(mask,cnt):
    	# list to store the points
	points = []

	# Get the moments from which the centroid of the contour can be calculated
	M = cv2.moments(cnt)
	cy = int(M['m10']/M['m00'])
	cx = int(M['m01']/M['m00'])
	#cx and cy are the x and y co-ordinates of the centroid

	points.append((cx,cy))

	return points

#Callback for the video data
def videoCallback(data):
	global frame
	ic = ImageConverter()
	frame = ic.convertImage(data)
	frame.setflags(write=1) # make image writable

#Function to extract 3D points from the point cloud
def extractPoints(data):
    depth_points = [] # List to store the 3D points
    for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=False): #Function to get the individual points
        depth_points.append((point[0],point[1],point[2]))

    return depth_points


#Function to transform the point
def transformPoint(point):
    w = 0.70711; x = -0.70711; y = 0; z = 0;
    q = Quaternion(w,x,y,z) # quaternion object for the rotation

    aPb_org = np.array([0.0006, 0.87375, -0.244]) # translation

    return q.rotate(point) + aPb_org


#Callback for the subsriber of the pointcloud
def depthCallBack(data):
    global frame

    lower_color = np.array([110, 50, 50])   #For blue
    upper_color = np.array([130 , 255 , 255])

    res,mask = pre_process(frame,lower_color,upper_color)
    #Obtain the contours
    index,contour = find_contour(mask)

    try:
        points = get_points(mask,contour[index])
        #Get the 3D points
        depth_points = extractPoints(data)

        for i in points:
            frame[i[0],i[1],0] = 255
            frame[i[0],i[1],1] = 255 # sets centroid to white color for better visualization
            frame[i[0],i[1],2] = 255

            #get the position in the 3D point list for the given 2D point
            pos = i[1] + i[0]*data.width
            #Get the x,y and z co-ordinates
            x = depth_points[pos][0]
            y = depth_points[pos][1]
            z = depth_points[pos][2]

            # transform the point to the frame of the base of the arm
            pickup_point = transformPoint(np.array([x, y, z]))

            # publish the pick up point
            pick_point.x = pickup_point[0]
            pick_point.y = pickup_point[1]
            pick_point.z = pickup_point[2]
        pub.publish(pick_point)
        print('published ', pick_point )

        #handle exceptions
    except Exception as e:
        # If no object detected, send NAN
        print("Object not detected")
        pick_point.x = float('nan')
        pick_point.y = float('nan')
        pick_point.z = float('nan')
        pub.publish(pick_point)
        print(e)
	#Draw the contours on the frame and show the images
    frame = cv2.drawContours(frame, contour, index, (0,0,255), 3)
    cv2.imshow('mask',mask)
    cv2.imshow('Image',frame)

    cv2.waitKey(5)


lower_color = np.array([110, 50, 50])   #For blue
upper_color = np.array([130 , 255 , 255])
try:
    rospy.init_node("point_calculator",anonymous = True) #initialize node
    rospy.Subscriber("/camera/color/image_raw",Image,videoCallback)#subsriber for the image
    rospy.Subscriber("/camera/depth/points",PointCloud2,depthCallBack)#subscriber for the point cloud

    pub = rospy.Publisher('/object_point_for_arm',Point, queue_size=10) # publisher for pickup point

    rospy.spin()# stop the python script from exiting

except Exception as e:
	print e
