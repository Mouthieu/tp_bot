#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# Variable globale pour détecter une collision
collision_detected = False  

def imu_callback(data):
    """Détecte une collision en fonction de l'accélération de l'IMU."""
    global collision_detected

    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y

<<<<<<< HEAD
    if (abs(ax)) >= 5 or (abs(ay)) >= 5:
        print("BOUM")
        return
    
=======
    # Seuil de collision (ajuste si nécessaire)
    if abs(ax) >= 5 or abs(ay) >= 5:
        rospy.logwarn("Collision détectée ! Commandes bloquées.")
        collision_detected = True
        stop_robot()

def stop_robot():
    """Envoie une commande de vitesse nulle pour arrêter le robot."""
    pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
    rospy.sleep(0.1)  # Pause pour s'assurer que le publisher est prêt

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.loginfo("Robot arrêté.")

def cmd_callback(msg):
    """Relaye les commandes de mouvement sauf en cas de collision."""
    global collision_detected
    pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

    if collision_detected:
        stop_robot()  # Empêche toute commande de mouvement
    else:
        pub.publish(msg)  # Relaye normalement les commandes

def reset_callback(msg):
    """Réinitialise le blocage après une collision."""
    global collision_detected
    if collision_detected:
        rospy.loginfo("Réinitialisation : le contrôle est réactivé.")
        collision_detected = False

>>>>>>> ff26b0373f28e6756926bfd4b36cfe89a2597a68
def listener():
    rospy.init_node('cmd_filter', anonymous=True)

    # Écoute l'IMU pour détecter les collisions
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)

    # Écoute les commandes de téléopération
    rospy.Subscriber("/teleop/cmd_vel", Twist, cmd_callback)

<<<<<<< HEAD
    print('Exiting node ' + rospy.get_name()) # This will only be executed if 'rospy.spin()' finishes, after having pressed Ctrl+c
=======
    # Écoute un topic pour réinitialiser après collision
    rospy.Subscriber("/reset_collision", Twist, reset_callback)

    rospy.spin()
>>>>>>> ff26b0373f28e6756926bfd4b36cfe89a2597a68

if __name__ == '__main__':
    listener()

