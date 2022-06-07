#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def laserMsg(msg):
    rospy.loginfo(msg.ranges[300])
    rospy.loginfo(msg.ranges[360])
    rospy.loginfo(msg.ranges[420])
    rospy.loginfo("--------------")

'''
    if(msg.ranges[360] < 1):
        move.linear.x = 0   
    else:
        move.linear.x = 1
    pub.publish(move)
'''
if __name__ == "__main__":
    rospy.init_node("laser_checks")
    sub = rospy.Subscriber("/base_scan_raw", LaserScan, laserMsg)
    pub = rospy.Publisher("/cmd_vel", Twist)
    move = Twist()

    rospy.spin()
