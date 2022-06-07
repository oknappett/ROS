#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math

last_reading = [0,0] #upper,lower
object_readings = 0
def LaserMessage(msg):
    global last_reading
    bot = msg.ranges[23] #top most scan of all shapes
    mid = msg.ranges[25] #middle scan 
    top = msg.ranges[28] #first scan above ground
    
    #find differences between scans
    upper = abs(top - mid)
    lower = abs(mid - bot)
    upper_dif = upper - last_reading[0]
    lower_dif = lower - last_reading[1]
    
    if(abs(upper - lower) < 0.05): #if the upper and lower sections are relatively equal
        if(upper_dif<0.1 and lower_dif<0.1):
            shape = "cube"
        else:
            shape = "cylinder"
    else:
        shape = "unknown"
    
    last_reading = [upper, lower]

    global object_readings
    object_detected = not math.isinf(msg.ranges[27])
    if(object_detected):
        rospy.loginfo(upper_dif)
        object_readings = object_readings + 1
        if(object_readings == 1):
            rospy.loginfo("detected "+ shape)
    else:
        object_readings = 0
        last_reading = [0,0]
        


if __name__ == "__main__":
    rospy.init_node("laser_scan")
    rospy.Subscriber("/velodyne/scan", LaserScan, LaserMessage)
    rospy.spin()




