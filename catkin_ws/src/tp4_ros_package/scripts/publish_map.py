#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt

def load_pgm_map(pgm_file, resolution=0.05, origin=(-10, -10, 0), negate=True):
    """ Charge une carte PGM et la convertit en OccupancyGrid. """
    img = cv2.imread(pgm_file, cv2.IMREAD_UNCHANGED)
    if img is None:
        rospy.logerr("Impossible de charger l'image PGM.")
        return None

    img = cv2.flip(img, 0)  # Inverser l'image verticalement pour correspondre à ROS
    # plt.imshow(img, cmap="gray", origin="upper")
    # plt.show()
    # img_display = np.where(img == -1, 205, img)  # -1 → gris clair pour affichage
    # img_display = (img_display * 255 / img_display.max()).astype(np.uint8)
    # cv2.imshow("Carte OccupancyGrid", img_display)

    # Convertir les valeurs en OccupancyGrid :
    # - 0 (noir) → obstacle (100)
    # - 255 (blanc) → libre (0)
    # - Valeurs intermédiaires → inconnues (-1)
    
    occupancy_data = np.zeros(img.shape, dtype=np.int8)

    # if negate:
    #     img = 255 - img  # Inversion des couleurs si nécessaire

    occupancy_data[img < 50] = 100  # Obstacles
    occupancy_data[img > 205] = 0   # Zones libres
    occupancy_data[(img >= 50) & (img <= 205)] = -120  # Inconnu

    return occupancy_data

def publish_map(map_data, resolution, origin):
    """ Publie la carte sous forme d'OccupancyGrid sur /map. """
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    rospy.init_node("map_publisher", anonymous=True)

    msg = OccupancyGrid()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.info.resolution = resolution
    msg.info.width = map_data.shape[1]
    msg.info.height = map_data.shape[0]

    msg.info.origin = Pose()
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.position.z = 0

    msg.data = map_data.flatten().tolist()

    rospy.loginfo("Carte publiée sur /map")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        #pgm_file = rospy.get_param("/image")  # Modifier avec le bon chemin
        pgm_file = "/home/mouthieu/catkin_ws/src/tp4_ros_package/map.pgm"
        resolution = 0.05  # Résolution de la carte en mètres/pixel
        origin = (-10, -10, 0)  # Origine de la carte (x, y, theta)
        
        map_data = load_pgm_map(pgm_file, resolution, origin)
        if map_data is not None:
            publish_map(map_data, resolution, origin)
    except rospy.ROSInterruptException:
        pass
