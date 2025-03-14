#!/usr/bin/env python
import rospy
import numpy as np
import sys
import sensor_msgs
import struct
import random
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
import tf

# Definition of class
class Estimation_Node:
    def __init__(self, node_name):
        self.nname = node_name 	#Giving a name for the ROS node

        rospy.init_node(self.nname, anonymous=True) #ROS node initialization

	self.num_of_plane_points = 100 # This sets a minimum number of points used to estimate a 3D plane

	self.plane_params = {"red":[-1]*4, "green":[-1]*4, "blue":[-1]*4} # A dictionnary holding the plane parameters, 4 per plane equation ax+by+cz+d = 0

	self.plane_points = {"red":[], "green":[], "blue":[]}

	self.feature_pose = Transform(Vector3(0, 0, 0.5), tf.transformations.quaternion_from_euler(0, 0, 0)) # This will hold the 6DOF pose of the feature, by a 3D vector for the translation and a quaternion for the rotation

	self.linear_solution = [] # This will hold the point of planes intersection obtained by solving a 3x3 linear system of equations
	
	point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.estimate_pose_callback) # ROS topic subscription

	self.br = tf.TransformBroadcaster()

	rospy.spin() # Initiate the ROS loop

    def empty_points(self):
	self.plane_points["red"] = []
	self.plane_points["green"] = []
	self.plane_points["blue"] = []

    def retrieve_points(self, color):
	return_array = []
	for _ in range(self.num_of_plane_points):
		point = random.randint(0, len(self.plane_points[color]))
		return_array.append(self.plane_points[color][point])
	to_np = np.array(return_array)
	return to_np

    def calculate_plane_equation(self, points):
	length = np.shape(points)[0]
	sol = []
	for _ in range(length):
		sol.append(-1)
	np_sol = np.array(sol)
	#left = np.linalg.inv(np.matmul(np.transpose(points), points))
	#mid = np.matmul(left, np.transpose(points))
	#return np.matmul(mid, np_sol)
	return np.linalg.solve(np.matmul(np.transpose(points), points), np.matmul(np.transpose(points), np_sol))


    def estimate_pose_callback(self, pointcloud_msg):
	#print 'Received PointCloud2 message. Reading data...'
	point_list = sensor_msgs.point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names = ("x", "y", "z", "rgb"))	

	#print 'Retrieving coordinates and colors...'
	for point in point_list:
	    rgb = struct.unpack('BBBB', struct.pack('f', point[3]))
	    #rospy.loginfo(point)

	    if rgb[2] > 100 and rgb[0] < 20 and rgb[1] < 20: # If dominant red point, concatenate it
		self.plane_points["red"] += [[point[0], point[1], point[2]]]
	    elif rgb[1] > 100 and rgb[0] < 20 and rgb[2] < 20: # If dominant green point, concatenate it
		self.plane_points["green"] += [[point[0], point[1], point[2]]]
	    elif rgb[0] > 100 and rgb[2] < 20 and rgb[1] < 20: # If dominant blue point, concatenate it
		self.plane_points["blue"] += [[point[0], point[1], point[2]]]

	# Test if there are sufficient points for each plane
	"""
	We will consider that 20 points is enough to make the plane
	"""
	if len(self.plane_points["red"]) < self.num_of_plane_points or len(self.plane_points["blue"]) < self.num_of_plane_points or len(self.plane_points["green"]) < self.num_of_plane_points:
		rospy.loginfo("Not enough points to make the plane")	

	# Estimate the plane equation for each colored point set using Least Squares algorithm
	red_points = self.retrieve_points("red")
	blue_points = self.retrieve_points("blue")
	green_points = self.retrieve_points("green")
	
	red_plan = self.calculate_plane_equation(red_points)  # Vecteur normal
	blue_plan = self.calculate_plane_equation(blue_points) # Vecteur normal
	green_plan = self.calculate_plane_equation(green_points) # Vecteur normal
	#rospy.loginfo("Plan rouge : {}".format(self.calculate_plane_equation(red_points)))
	#rospy.loginfo("Plan bleu : {}".format(self.calculate_plane_equation(blue_points)))
	#rospy.loginfo("Plan vert : {}".format(self.calculate_plane_equation(green_points)))
	#A = np.array([red_plan[:3], blue_plan[:3], green_plan[:3]])
        #b = np.array([-red_plan[3], -blue_plan[3], -green_plan[3]])
	#intersection_point = np.linalg.solve(A, b)
        #rospy.loginfo("Intersection point: {}".format(intersection_point))	

	### Enter your code ###

	# Verify that each pair of 3D planes are approximately orthogonal to each other
	### Enter your code ###
	""" ---OK---
	rospy.loginfo("Produit scalaire entre rouge et bleu : {}".format(np.dot(np.transpose(blue_plan), red_plan)))
	rospy.loginfo("Produit scalaire entre rouge et vert : {}".format(np.dot(np.transpose(green_plan), red_plan)))
	rospy.loginfo("Produit scalaire entre vert et bleu : {}".format(np.dot(np.transpose(blue_plan), green_plan)))
	"""

	# Feature detection
	# Solve 3x3 linear system of equations given by the three intersecting planes, in order to find their point of intersection
	### Enter your code ###
	m = np.concatenate(([red_plan], [blue_plan], [green_plan]))
	corner = np.linalg.solve(m, [-1, -1, -1])
	rospy.loginfo(corner)

	# Obtain z-axis (blue) vector as the vector orthogonal to the 3D plane defined by the red (x-axis) and the green (y-axis)
	### Enter your code ###
	blue_vector = np.cross(red_plan, green_plan)
	blue_vector = blue_vector/np.linalg.norm(blue_vector)

	# Obtain y-axis (green) vector as the vector orthogonal to the 3D plane defined by the blue (z-axis) and the red (x-axis)
	### Enter your code ###
	green_vector = np.cross(blue_vector, red_plan)
	green_vector = green_vector/np.linalg.norm(green_vector)

	# red
	red_vector = np.cross(blue_vector, green_vector)
	red_vector = red_vector/np.linalg.norm(red_vector)

	# Construct the 3x3 rotation matrix whose columns correspond to the x, y and z axis respectively
	### Enter your code ###
	rot_m = np.concatenate(([red_vector], [green_vector], [blue_vector]))
	rot_m = np.transpose(rot_m)

	# Obtain the corresponding euler angles from the previous 3x3 rotation matrix
	### Enter your code ###
	euler_angles = tf.transformations.euler_from_matrix(rot_m)

	# Set the translation part of the 6DOF pose 'self.feature_pose'
	self.feature_pose.translation.x = corner[0]
	self.feature_pose.translation.y = corner[1]
	self.feature_pose.translation.z = corner[2]

	# Set the rotation part of the 6DOF pose 'self.feature_pose'
	self.feature_pose.rotation = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])

	# Publish the transform using the data stored in the 'self.feature_pose'
	self.br.sendTransform((self.feature_pose.translation.x, self.feature_pose.translation.y, self.feature_pose.translation.z), self.feature_pose.rotation, rospy.Time.now(), "corner_6dof_pose", "camera_depth_optical_frame") 

	# Empty points
        self.empty_points()

if __name__ == '__main__':
    my_estim_object = Estimation_Node('my_estimation_node')
