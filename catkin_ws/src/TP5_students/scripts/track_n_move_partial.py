#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import cv2
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

"""
Hints:
    Aruco markers:
        https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/Aruco.html
    Histogram Backprojection
        https://docs.opencv.org/3.4/dc/df6/tutorial_py_histogram_backprojection.html
    Meanshift & Camshift:
        https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_meanshift/py_meanshift.html
"""

class Node:

    def __init__(self, type_):
        """
        Initialization of aproximated synchronized subscription to topics.
        """
        self.mode = type_

        rospy.init_node('track_n_move')
        # Topic subscription
        rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

        # Synchronization of received messages
        ats = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.callback)
        # ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 1)
        # ts.registerCallback(self.callback)

        # OpenCV bridge to convert ROS image into OpenCV image
        self.bridge = CvBridge()

        # Publisher of a velocity commands        
        self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        # Publisher of a processed image
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        
        # Creating aruco_dict with 5x5 bits with max 250 ids
        # Ids ranges from 0-249
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)

        # Velocity control message
        self.msg_vel = Twist()
        rospy.spin()

    def detect_aruco(self, img):
        """
        Wrapper around  OpenCV detectMarkers function
        Nothing to do here.
        :param img: cv image
        :return: list(dict(id: list(corners),...))
        """
        aruco_list = {}
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # lists of ids and the corners beloning to each id
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if len(corners):
            for k in range(len(corners)):
                temp_1 = corners[k]
                temp_1 = temp_1[0]
                temp_2 = ids[k]
                temp_2 = temp_2[0]
                aruco_list[temp_2] = temp_1
            return aruco_list

    def mark_Aruco(self, img, aruco_list):
        """
        Nothing to do here.
        :param img: opencv image
        :param aruco_list: list of detected aruco markers
        :return img: opencv image with drawn marker centers
        :return centers: centers of aruco markers
        """
        key_list = aruco_list.keys()
        centers = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        for key in key_list:
            dict_entry = aruco_list[key]    
            centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
            centre[:] = [int(x / 4) for x in centre] 
            orient_centre = centre + [0.0,5.0]
            centre = tuple(centre)  
            orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
            centers.append(list(centre))
            cv2.circle(img,centre,1,(0,0,255),8)
            cv2.line(img,centre,orient_centre,(255,0,0),4) 
        return img, centers

    def callback(self, rgb, depth):
        """
        This transforms a ROS image into an OpenCV image, 
        calls for a corresponding processing method and publishes a processed ROS image.
        Nothing to do here.
        :param data: ROS RGB and depth images
        """
        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            rospy.loginfo('ROS img -> cv2 img conversion failure')
            return None

        if self.mode == "circle":
            # Part I: red circle
            cv_image = self.process_image_circle(cv_rgb_image, cv_depth_image)
        elif self.mode == "joker":
            # Part II: a random image
            cv_image = self.process_random_image(cv_rgb_image, cv_depth_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.loginfo('cv2 img -> ROS img conversion failure')

    def process_random_image(self, img, depth):
        """
        Processing of image with a random picture
        Hint 1: maybe, you will need to add a source of light in Gazebo.
                It is on the top panel.
        Hint 2: probably, it is worth to move the robot somehow. 
        :param img: opencv image
        :return img: processed opencv image
        """
        try:
            # Detect Aruco markers
            aruco_list = self.detect_aruco(img)
            # Draw their centers and find their centers
            img, centers = self.mark_Aruco(img, aruco_list)
        except Exception as e:
            print("No Aruco markers in the field of view.\n")
            return img

        # Your code starts here

        # Transform into hsv
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)	

        # Publish commands
        linear = 0.
        angular = 0.
        self.send_commands(linear, angular)        
        return img

    def process_image_circle(self, img, depth):
	    """
	    Processing of an image with a circle.
	    :param img: opencv image (in HSV)
	    :return img: processed opencv image
	    """
	    # Convertir l'image en niveaux de gris (en utilisant uniquement la composante de valeur, V)
	    gray = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)  # Convertir en BGR pour un meilleur rendu en niveaux de gris
	    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

	    # Appliquer un flou médian pour réduire le bruit
	    gray = cv2.medianBlur(gray, 5)

	    # Appliquer la transformation de Hough pour détecter les cercles
	    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
		                        param1=50, param2=30, minRadius=0, maxRadius=0)
	    #print(circles)
	    # Vérifier si des cercles sont détectés
	    if circles is not None:
		circles = np.uint16(np.around(circles))  # Convertir les coordonnées des cercles en entiers

		# Dessiner les cercles et leurs centres sur l'image
		for i in circles[0, :]:
		    # Dessiner le cercle extérieur
		    cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
		    # Dessiner le centre du cercle
		    cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)

		# Initialiser la fenêtre de suivi à partir du premier cercle détecté
		x, y, radius = i[0], i[1], i[2]
		# Vérification pour éviter les indices négatifs
		x = max(x - radius, 0)
		track_window = (x, y, 2 * radius, 2 * radius)

		# Convertir l'image en HSV pour appliquer MeanShift
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# S'assurer que les indices sont dans les limites de l'image
		roi_x1 = max(x, 0)
		roi_y1 = max(y, 0)
		roi_x2 = min(x + 2 * radius, img.shape[1])  # largeur maximale de l'image
		roi_y2 = min(y + 2 * radius, img.shape[0])  # hauteur maximale de l'image
		roi = hsv[roi_y1:roi_y2, roi_x1:roi_x2]

		roi_hist = cv2.calcHist([roi], [0], None, [180], [0, 180])
		cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

		# Calculer le backprojection pour MeanShift
		dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

		# Appliquer MeanShift pour le suivi du cercle
		ret, track_window = cv2.meanShift(dst, track_window, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))
		print("_____________________________________________________")		
		print(track_window)
		# Si le suivi est réussi, dessiner la nouvelle position
		if ret:
		    x, y, w, h = track_window
		    img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
		    cv2.putText(img, "Tracking Active", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		else:
		    # Si le suivi échoue, afficher un message d'erreur
		    cv2.putText(img, "Tracking Failed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

	    else:
		# Si aucun cercle n'est détecté, afficher un message d'erreur
		cv2.putText(img, "No Circle Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

	    # Retourner l'image avec les annotations visuelles

	    # Publish necessary commands
	    window_center = track_window[0]+track_window[2]/2
	    camera_center = 280

	    delta = window_center - camera_center
	    angular = 0
	    linear = 0
	    alpha = 0.2

	    if abs(delta)<50:
		angular = 0
	    else:
		if delta>0:
			angular=-alpha
		else:
			angular=alpha
	    
            #linear = 0.1

	    self.send_commands(linear, angular)


	    return img




        
    def send_commands(self, linear, angular):
        """
        Nothing to do here.
        :param cmds: dictionary of velocity commands
        """
        self.msg_vel.linear.x = linear
        self.msg_vel.angular.z = angular
        self.pub_vel.publish(self.msg_vel)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Looking for a circle")
        Node("circle")
    else:
        print("Looking for a {}".format(sys.argv[1]))
        Node(sys.argv[1])

