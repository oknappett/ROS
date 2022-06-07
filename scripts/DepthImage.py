#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2

bridge = CvBridge()

def cb_depthImage(image):
    global bridge
    global y
    global x
    # image msg obtained from callback message for topic
    try:
        cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
        
        # where x,y is the centre point from the published moment
        depth = cv_image[y][x]
        rospy.logdebug('Depth of point is %s m',depth)
        
        # For testing/verification:
        cv2.circle(cv_image, (x,y), 10, 255) # draw circle radius 10 at x,y
        cv2.imshow("Image window", cv_image) # display the image
        cv2.waitKey(3)

    except CvBridgeError as e:
        print(e)


if __name__ == "__main__":
    rospy.init_node("depthTest", anonymous = False)
    rospy.Subscriber("/head_camera/depth_registered/image_raw", Image, cb_depthImage)
    rospy.spin()
        


