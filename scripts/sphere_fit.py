#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision_lectures.msg import XYZarray 
from robot_vision_lectures.msg import SphereParams



xyz_data = XYZarray()
sphere_data = SphereParams()
sphere_computed = False


def compute_model(data):
	
	global points
	global sphere_data
	global sphere_computed

	points = data.points
	
	#compute matrices A and B 
	A = []
	B = []
	for i in range(len(points)):
		A2 = []
		A2.append(2*points[i].x)
		A2.append(2*points[i].y)
		A2.append(2*points[i].z) 
		A2.append(1)
		A.append(A2)
		B.append(points[i].x**2 + points[i].y**2 + points[i].z**2)
	

	#compute P
	P = np.linalg.lstsq(A, B, rcond=None)[0]
	#compute radius
	r = np.sqrt(P[0]**2 + P[1]**2 + P[2]**2 + P[3])
	
	#assign coordinates and radius
	sphere_data.xc = P[0]
	sphere_data.yc = P[1]
	sphere_data.zc = P[2]
	sphere_data.radius = r
	
	sphere_computed = True

if __name__ == '__main__':
	# define the node
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to read images
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, compute_model) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		if sphere_computed: 
			img_pub.publish(sphere_data)
		rate.sleep()
