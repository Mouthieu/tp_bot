#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

# Function which is called when a ROS message is heard in the respective ROS topic (see Subscriber initialization further down). The message is stored in the function's argument 'data'

acceleration_data = None

def callback(data):
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z

    if (abs(ax)) >= 5 or (abs(ay)) >= 5:
	print("BOUM")
	return
    
def listener():
    global accelration_data
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off.
    rospy.init_node('listener_node', anonymous=True)

    # The node will subscribe to a ROS topic named 'talking_topic' and whenever a ROS message is heard there, the 'callback' function will be executed. 
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, callback)

    #rospy.spin() keeps python from exiting until the node is stopped (by pressing Ctrl+c for example). Also, by entering this infinite loop, it allows any ROS callback functions to be executed, whenever a message is listened from the respective ROS topic. The variant function 'rospy.spinOnce()' would allow the callbacks to be executed only once and after exit.
    rospy.spin()

    

    print 'Exiting node ' + rospy.get_name() # This will only be executed if 'rospy.spin()' finishes, after having pressed Ctrl+c

if __name__ == '__main__':
    listener()
    
