#!/usr/bin/env python

#import python libraries
import rospy
import actionlib
import time
import math
#import movement libraries
from opencv_apps.msg import MomentArrayStamped
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Pose, Point, Quaternion
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import *
#import tf libraries
import tf2_ros
import tf_conversions
#import peripheral libraries
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2

turnOffset = ""
def LaserMessage(msg):
    """callback for obstacle avoidance
    moves robot left or right depending on where object is infront
    """
    global turnOffset
    
    right = msg.ranges[540]
    left = msg.ranges[180]
    rospy.loginfo(2)
    if((left<2)):
        rospy.loginfo("object on left")
        turnOffset = "object on left"
    elif((right < 2)):
        rospy.loginfo("object on right")
        turnOffset = "object on right"
    else:
        turnOffset = "none"


rospy.init_node("Objects_in_bin", anonymous = True)
rospy.Subscriber("/base_scan", LaserScan, LaserMessage)
lol = True
while(lol):
    rospy.loginfo(turnOffset)
