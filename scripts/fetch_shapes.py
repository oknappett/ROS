#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw

import random
import rospy
from geometry_msgs.msg import Twist
import math

side_length = 2
num_corners = 4
angle = 90
speed = 0.5


def pubvel():
	# Initialise the ROS system and become a node
    rospy.init_node('publish_velocity')
	
	# Create a publisher object
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
	
	# Loop at 2Hz until the node is shutdown
    rate = rospy.Rate(25)
   
    global angle
    global num_corners
    global side_length
    speed = 0.1
    corners = 0
    Turn_angle = (math.pi/180)*angle 
    start_time = rospy.get_time()
    state = "forward"
    while not rospy.is_shutdown(): 
        msg = Twist()
        now = rospy.get_time()
        if(corners == num_corners):
            msg.linear.x = 0
            msg.angular.z = 0
            rospy.loginfo("shape finished")
            pub.publish(msg)
            break
        else:
            if(state == "forward"):
                distance_travelled = (now - start_time) * speed
                if(distance_travelled > side_length):
                    msg.linear.x = 0
                    state = "turn"
                    distance_travelled = 0
                    start_time = rospy.get_time()
                else:
                    msg.linear.x = speed
            elif(state == "turn"):
                distance_turned = ((now - start_time) * speed) *(180/math.pi)
                if(distance_turned > Turn_angle):
                    corners = corners + 1
                    msg.angular.z = 0
                    state = "forward"
                    disance_turned = 0
                    start_time = rospy.get_time()
                else:
                    msg.angular.z = speed
        
        pub.publish(msg)
        #rospy.loginfo(distance) 
        rate.sleep()
    


if __name__ == '__main__':
    pubvel()
	
