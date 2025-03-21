#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tty
import sys
import select
import termios
from geometry_msgs.msg import Twist

# Definition of class
class Teleoperation_Node:
    def __init__(self, node_name):
	"""
	This method is launched whenever we create a new instance of the class
	Parameters:
		node_name: The name of the name we want to initialize
	Returns:
		None
	"""
        rospy.init_node(node_name, anonymous=True)  # Remplacer le nom du nœud ici
        # Create a publisher to send messages to /cmd_vel_mux/input/navi
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Initial robot velocities
        self.linear_speed = 0.5  # Initial linear speed (m/s)
        self.angular_speed = 0.5  # Initial angular speed (rad/s)

    def getKey(self):
	"""
	This method reads a single keyboard character from the terminal and returns this character
	Parameters:
		None	
	Returns:
		key: The key we pressed on the keyboard
    	"""
        # Back-up default terminal settings
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())  # Setting stdio terminal to raw (no need for pressing enter)
        key = sys.stdin.read(1)  # Read 1 character 

        # Restore default terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def control_robot(self):
	"""
	This method makes the link between the actions we make on the keyboard and the data sent to the topic in order to make the robot move
	Parameters:	
		None
	Returns:
		None
	"""
        move_cmd = Twist()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            key = self.getKey()

            if key == 'u':  # Move forward
                move_cmd.linear.x = self.linear_speed
                move_cmd.angular.z = 0
            elif key == 'j':  # Move backward
                move_cmd.linear.x = -self.linear_speed
                move_cmd.angular.z = 0
            elif key == 'k':  # Rotate clockwise
                move_cmd.linear.x = 0
                move_cmd.angular.z = -self.angular_speed
            elif key == 'h':  # Rotate counter-clockwise
                move_cmd.linear.x = 0
                move_cmd.angular.z = self.angular_speed
            elif key == 'f':  # Increase speed
                self.linear_speed *= 1.1  # Increase linear speed by 10%
                self.angular_speed *= 1.1  # Increase angular speed by 10%
                rospy.loginfo("Speed increased: Linear = {}, Angular = {}".format(self.linear_speed, self.angular_speed))
            elif key == 's':  # Decrease speed
                self.linear_speed *= 0.9  # Decrease linear speed by 10%
                self.angular_speed *= 0.9  # Decrease angular speed by 10%
                rospy.loginfo("Speed decreased: Linear = {}, Angular = {}".format(self.linear_speed, self.angular_speed))
            elif key == 'q':  # Quit the program
                break  # Exit the loop and terminate the node

            else:  # Stop the robot when no relevant key is pressed
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0

            # Publish the message
            self.pub.publish(move_cmd)

            # Sleep to maintain the loop rate
            rate.sleep()

if __name__ == '__main__':
    try:
        # Create an instance of Teleoperation_Node and start controlling the robot
        teleop = Teleoperation_Node('teleoperation_node')  # The name of the node
        teleop.control_robot()  # Start the control loop
    except rospy.ROSInterruptException:
        pass

