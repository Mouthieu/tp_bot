#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tty
import sys
import select
import termios
from geometry_msgs.msg import Twist

class Teleoperation_Node:
    def __init__(self, node_name):
	"""
	This method is launched whenever we create a new instance of the class
	Parameters:
		node_name: The name of the name we want to initialize
	Returns:
		None
	"""
        rospy.init_node(node_name, anonymous=True)

        # Get the parameters from the .launch file
        self.linear_speed = rospy.get_param('linear_speed', 0.5)
        self.angular_speed = rospy.get_param('angular_speed', 1)
        
        # Gather the actions of the keys
        self.key_u = rospy.get_param('key_u', 'forward')
        self.key_j = rospy.get_param('key_j', 'backward')
        self.key_k = rospy.get_param('key_k', 'rotate_right')
        self.key_h = rospy.get_param('key_h', 'rotate_left')
        self.key_f = rospy.get_param('key_f', 'increase_linear_speed')
        self.key_s = rospy.get_param('key_s', 'decrease_linear_speed')
	self.key_r = rospy.get_param('key_r', 'increase_angular_speed')
        self.key_z = rospy.get_param('key_z', 'decrease_angular_speed')

        # Publisher to send commands
        self.pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)

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
        key = sys.stdin.read(1) # Read 1 character 

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
            key = self.getKey() # Returns the pressed key

            if key == 'u' and self.key_u == 'forward':  # Avancer
                move_cmd.linear.x = self.linear_speed
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = 0
            elif key == 'j' and self.key_j == 'backward':
                move_cmd.linear.x = -self.linear_speed
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = 0
            elif key == 'k' and self.key_k == 'rotate_right':
                move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = -self.angular_speed
            elif key == 'h' and self.key_h == 'rotate_left':
                move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = self.angular_speed
            elif key == 'f' and self.key_f == 'increase_linear_speed':
                self.linear_speed *= 1.1 # Increase linear speed by 10%
                

		move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = 0
                rospy.loginfo("Speed increased: Linear = {}, Angular = {}".format(self.linear_speed, self.angular_speed))
            elif key == 's' and self.key_s == 'decrease_linear_speed':
                self.linear_speed *= 0.9  # Decrease linear speed by 10%
                

		move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = 0
                rospy.loginfo("Speed decreased: Linear = {}, Angular = {}".format(self.linear_speed, self.angular_speed))
	    elif key == 'r' and self.key_r == 'increase_angular_speed':
                self.angular_speed *= 1.1 # Decrease angular speed by 10%
                

		move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = 0
                rospy.loginfo("Speed increased: Linear = {}, Angular = {}".format(self.linear_speed, self.angular_speed))
	    elif key == 'z' and self.key_z == 'decrease_angular_speed':
                self.angular_speed *= 0.9 # Decrease angular speed by 10%
                

		move_cmd.linear.x = 0
		move_cmd.linear.y = 0
		move_cmd.linear.z = 0

		move_cmd.angular.x = 0
		move_cmd.angular.y = 0
                move_cmd.angular.z = 0
                rospy.loginfo("Speed increased: Linear = {}, Angular = {}".format(self.linear_speed, self.angular_speed))
            elif key == 'q':  # Quit the program
                break # Exit the loop and terminate the node

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
        teleop = Teleoperation_Node('teleoperation_node') # The name of the node
        teleop.control_robot() # Start the control loop
    except rospy.ROSInterruptException:
        pass
