#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision_lectures.msg import XYZarray 
from robot_vision_lectures.msg import SphereParams



xyz_data = XYZarray
sphere_data = SphereParams

sphere_computed = False

# get the image message
def get_points(data):
	A_B(data.points)
	
	
def A_B(points):
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
	
	compute_sphere(A,B)
	
	
def compute_sphere(A,B):
	p = np.linalg.lstsq(A, B, rcond=None)[0]
	r = np.sqrt(p[0]**2 + p[1]**2 + p[2]**2 + p[3])
	sphere_data.xc = p[0]
	sphere_data.yc = p[1]
	sphere_data.zc = p[2]
	sphere_data.radius = r
	sphere_computed = True

if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_points) 
	# define a publisher to publish images
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		if sphere_computed: 
			sphere_pub.publish(sphere_data)
		rate.sleep()
