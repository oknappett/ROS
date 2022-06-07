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


def InitialPose():
    """initialises pose
    head camera straight ahead
    arm out and up to left
    """
    tiltHead([0,0])
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # TF joint names
    joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    # Lists of joint angles in the same order as in joint_names
    pose = [0.0, 1.5, 0, 3.0, 1.0, 3.0, 1.0, 0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joint_names, pose, wait=False)

    # Since we passed in wait=False above we need to wait here
    move_group.get_move_action().wait_for_result()
    result = move_group.get_move_action().get_result()

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Pose initialised!")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")

    


#globals
x=y=area=0
imageHeight = 480
imageWidth = 640 
arrayLength = 0
state = "searching"
turnVel = 0
def moment(msg):
    """callback function for moment messages
    assigns the values in the moment to globals
    """
    moments = msg.moments
    global arrayLength
    arrayLength = len(moments)
    if(len(moments)>0):
        global x
        global y
        global area
        
        x = moments[0].center.x
        y = moments[0].center.y
        area = moments[0].area

#global
turnOffset = 0
def LaserMessage(msg):
    """callback for obstacle avoidance
    moves robot left or right depending on where object is infront
    """
    global turnOffset
    right = msg.ranges[540]
    left = msg.ranges[180]
    
    if(state == "approach"):
        if((left<2)):
            turnOffset = -0.4
        elif((right < 2)):
            turnOffset = 0.4
        else:
            turnOffset = 0
        
        
def findRed():
    """Moment searches for red colour
    uses proxy service to change dynamic reconfigure in parameter to red values
    """
    rospy.wait_for_service("/rgb_color_filter/set_parameters")

    dynam_serv = rospy.ServiceProxy("/rgb_color_filter/set_parameters", Reconfigure)
    config = Config()
    r_max_param = IntParameter()
    r_max_param.name = "r_limit_max"
    r_max_param.value = 255
    config.ints.append(r_max_param)
    r_min_param = IntParameter()
    r_min_param.name = "r_limit_min"
    r_min_param.value = 100
    config.ints.append(r_min_param)
    b_max_param = IntParameter()
    b_max_param.name = "b_limit_max"
    b_max_param.value = 50
    config.ints.append(b_max_param)
    b_min_param = IntParameter()
    b_min_param.name = "b_limit_min"
    b_min_param.value = 0
    config.ints.append(b_min_param)
    g_max_param = IntParameter()
    g_max_param.name = "g_limit_max"
    g_max_param.value = 50
    config.ints.append(g_max_param)
    g_min_param = IntParameter()
    g_min_param.name = "g_limit_min"
    g_min_param.value = 0
    config.ints.append(g_min_param)
    response = dynam_serv(config)

def findBlue():
    """Moment searches for blue colour
    uses proxy service to change dynamic reconfigure in parameter to blue values
    """
    rospy.wait_for_service("/rgb_color_filter/set_parameters")

    dynam_serv = rospy.ServiceProxy("/rgb_color_filter/set_parameters", Reconfigure)
    config = Config()
    r_max_param = IntParameter()
    r_max_param.name = "r_limit_max"
    r_max_param.value = 50
    config.ints.append(r_max_param)
    r_min_param = IntParameter()
    r_min_param.name = "r_limit_min"
    r_min_param.value = 0
    config.ints.append(r_min_param)
    b_max_param = IntParameter()
    b_max_param.name = "b_limit_max"
    b_max_param.value = 255
    config.ints.append(b_max_param)
    b_min_param = IntParameter()
    b_min_param.name = "b_limit_min"
    b_min_param.value = 100
    config.ints.append(b_min_param)
    g_max_param = IntParameter()
    g_max_param.name = "g_limit_max"
    g_max_param.value = 50
    config.ints.append(g_max_param)
    g_min_param = IntParameter()
    g_min_param.name = "g_limit_min"
    g_min_param.value = 0
    config.ints.append(g_min_param)

    response = dynam_serv(config)
        

#globals
bridge = CvBridge()
depth = 0
def cb_depthImage(image):
    """find depth of head camera image
    uses coords of moment to find depth in head camera 
    puts values in tf tree with offset from camera
    """
    global bridge, x, y, depth, imageWidth, imageHeight
    try:
        imgy = int(y)
        imgx = int(x)
        cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
        depth = cv_image[imgy][imgx]
        #rospy.loginfo("Depth is %s m", depth)
        
        
        #calculate offsets
        f = 554.254691191187
        offsetX = -((imgx - (imageWidth/2))/f)*depth
        offsetY = -((imgy - (imageHeight/2))/f)*depth
        
        #broadcast to tree
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "head_camera_depth_frame"
        t.child_frame_id = "target_object"
        t.transform.translation.x = depth
        t.transform.translation.y = offsetX
        t.transform.translation.z = offsetY
        
        q = tf_conversions.transformations.quaternion_from_euler(0,0,0) 
        t.transform.rotation.x = q[0]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
        

        cv2.circle(cv_image, (imgx,imgy), 10, 255)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    except CvBridgeError as e:
        print(e)

def tiltHead(pose):
    """tilts head
    param: pose -> pose coords of head [pan, tilt]
    """
    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected")     
    
    trajectory = JointTrajectory()
    trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]

    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = pose
    trajectory.points[0].velocities = [0.0] * len(pose)
    trajectory.points[0].accelerations = [0.0] * len(pose)
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    head_goal = FollowJointTrajectoryGoal()
    head_goal.trajectory = trajectory
    head_goal.goal_time_tolerance = rospy.Duration(0.0)
    rospy.loginfo("Setting head pose...")
    head_client.send_goal(head_goal)

    head_client.wait_for_result(rospy.Duration(6.0))       
    rospy.loginfo("...done")     



def turnRobot():
    """finite state machine for main behaviour
    picks up blue objects and puts them in/on a red one
    """
    
    rospy.init_node("Objects_in_bin", anonymous = True)
    
    rospy.Subscriber("/contour_moments/moments", MomentArrayStamped, moment)
    rospy.Subscriber("/head_camera/depth_registered/image_raw", Image, cb_depthImage)
    rospy.Subscriber("/base_scan", LaserScan, LaserMessage)
    global x
    global y
    global area 
    global depth
    global arrayLength
    global state
    global turnVel
    global imageWidth
    global imageHeight

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)    
    
    rate = rospy.Rate(10)

    InitialPose()
    
    gripper_closed = 0.00
    gripper_open = 0.1        

    rospy.loginfo("Waiting for gripper_controller...")
    gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper_client.wait_for_server()
    rospy.loginfo("...connected.")

    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 10.0
    gripper_goal.command.position = gripper_open

    rospy.loginfo("Setting positions closed...")
    gripper_client.send_goal(gripper_goal)
    gripper_client.wait_for_result(rospy.Duration(5.0))

    time.sleep(1.0)
    rospy.loginfo("done")

    
    #pose_stamped message
    move_group = MoveGroupInterface('arm_with_torso', 'base_link')
    gripper_frame = 'wrist_roll_link'
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    #define planning objects
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")

    planning_scene.removeCollisionObject("under_object")
    planning_scene.removeCollisionObject("above_object")   
    planning_scene.removeCollisionObject("right_of_object")
    planning_scene.removeCollisionObject("left_of_object")
    planning_scene.removeCollisionObject("side_object_left")
    planning_scene.removeCollisionObject("side_object_right")    

    #initialise buffer
    
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    state = "colourPick"
    holding_cube = False
    start_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        msg = Twist()
        now_time = rospy.get_time()
        #fsm for getting the cube or dropping cube off
        if(state == "colourPick"):
            '''
            state to choose colour filters for the moment message
            '''
            if(holding_cube):
                findRed()
            elif(not holding_cube):
                findBlue()
            state = "searching"

        elif(state == "searching"):
            '''
            state to search for object
            spins until moment message appears
            '''
            spin_time = now_time - start_time
            if(spin_time>21.5):
                #done a whole 360 spin search without finding cube/bin
                break
            
            msg.linear.x = 0
            if(arrayLength == 0):
                msg.angular.z = 0.5
            else:
                msg.angular.z = 0
                state = "approach"
        
        elif(state == "approach"):
            '''
            state to approach object
            turnOffset is the obstacle avoidance
            moves towards object with PID controller to keep object centred
            moves onto next state when object is <2m
            '''   
            msg.linear.x = 0.3
            if(depth < 2):
                msg.linear.x = 0.3
                msg.angular.z = 0
                state = "torso_changes"     
            else:
                if(y < (imageHeight/5)):
                    headPose = [0,0.7]
                    tiltHead(headPose)
                else:
                    if(x < (imageWidth/2) - 25):
                        #realigning right
                        msg.angular.z = 0.4 
                    elif(x > (imageWidth/2) + 25):
                        #Realigning left
                        msg.angular.z = -0.4 
                    elif((x>((imageWidth/2)-25)) and (x<(imageWidth/2)+25)):               
                        #centre
                        msg.linear.x = 0.3
                        #realign for drift in gazebo
                        msg.angular.z = -0.03 
            
                    msg.angular.z = msg.angular.z + turnOffset
                #rospy.loginfo(msg.angular.z)
                #rospy.loginfo(turnOffset)
                #rospy.loginfo("====")

        elif(state == "torso_changes"):
            '''
            state to change how lifted up the torso is
            if the object is high up, robot lifted all the way
            '''
            joint_names = ["torso_lift_joint", "shoulder_pan_joint","shoulder_lift_joint", "upperarm_roll_joint","elbow_flex_joint", "forearm_roll_joint","wrist_flex_joint", "wrist_roll_joint"]
            if(y<260):
                # Lists of joint angles in the same order as in joint_names
                pose = [0.4, 1.5, 0, 3.0, 1.0, 3.0, 1.0, 0]
            else:
                  pose = [0.0, 1.5, 0, 3.0, 1.0, 3.0, 1.0, 0]

            # Plans the joints in joint_names to angles in pose
            move_group.moveToJointPosition(joint_names, pose, wait=False)
            # Since we passed in wait=False above we need to wait here
            move_group.get_move_action().wait_for_result()
            result = move_group.get_move_action().get_result()
            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Pose initialised!")
                else:
                    rospy.logerr("torso goal in state: %s", move_group.get_move_action().get_state())
            else:
                    rospy.logerr("MoveIt! failure no result returned.")
            
            state = "head_tilt_down"
                   
        elif(state == "head_tilt_down"):
            '''
            tilts head to look at object
            looks further down when holding cube because bin is on the floor, not a table
            '''
            if(not holding_cube):
                pose = [0,0.5]
            elif(holding_cube):
                pose = [0,0.7]
            tiltHead(pose)    
            state = "centre_object"
    
        elif(state == "centre_object"):
            '''
            simple PID controller to centre object before moving next to object
            '''
            if(x < (imageWidth/2) - 25):
                #realigning right
                msg.angular.z = 0.4
            elif(x > (imageWidth/2) + 25):
                #Realigning left
                msg.angular.z = -0.4
            elif((x>((imageWidth/2)-25)) and (x<(imageWidth/2)+25)):               
                #centre
                msg.linear.x = 0
                msg.angular.z = 0
                state = "move_to_object"

        elif(state == "move_to_object"):
            '''
            moves robot right up to robot
            moves so object is 85cm away when not holding an object
            bin is on floor again so needs to be further away due to height of robot
            '''
            msg.linear.x = 0.15
            #align z for drift
            msg.angular.z = -0.15
            if(x < (imageWidth/2) - 25):
                #realigning right
                msg.angular.z = 0.4
            elif(x > (imageWidth/2) + 25):
                #Realigning left
                msg.angular.z = -0.4
            elif((x>((imageWidth/2)-25)) and (x<(imageWidth/2)+25)):
                #centre
                msg.angular.z = 0
            if(not holding_cube):   
                if(depth < 0.75): 
                    msg.linear.x = 0
                    state = "read_transform"
            elif(holding_cube):
                if(depth < 0.95):
                    msg.linear.x = 0 
                    state = "read_transform"

        elif(state == "read_transform"):
            '''
            reads pose of target object from tf tree and assigns to variables
            adds collision objects around target object using these values
            '''
            msg.linear.x=0
            msg.angular.z = 0
            #read object location from tf
            try:
                trans = tfBuffer.lookup_transform("base_link", "target_object", rospy.Time())
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            transx = trans.transform.translation.x
            transy = trans.transform.translation.y
            transz = trans.transform.translation.z
            
            #roll x, pitch y, yaw z, w
            rotx = trans.transform.rotation.x
            roty = trans.transform.rotation.y 
            rotz = trans.transform.rotation.z
            rotw = trans.transform.rotation.w

            planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
            planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
            planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
            planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    
            if(not holding_cube):
                planning_scene.addCube("left_of_object", 0.5, (transx-0.1), (transy + 0.4), (transz+0.1))
                planning_scene.addCube("right_of_object", 0.5, (transx-0.1), (transy-0.5), (transz+0.1))
                planning_scene.addCube("under_left_of_object", 0.5, (transx-0.2), (transy + 0.4), (transz/2))
                planning_scene.addCube("under_right_of_object", 0.5, (transx-0.2), (transy-0.5), (transz/2))
                planning_scene.addCube("under_object", transz, (transx+(transz/2)-0.2), transy , ((transz/2)-0.1))
                planning_scene.addCube("above_object", 0.7, (transx+0.1), transy , (transz+0.8))
                #planning scene to not swipe object away
                planning_scene.addCube("side_object_right", 0.05, (transx), (transy - 0.15), transz)
                planning_scene.addCube("side_object_left", 0.05, (transx-0.2), (transy + 0.1), transz)
            elif(holding_cube):
                planning_scene.addCube("left_of_object", 0.5, (transx), (transy+0.5), (transz-0.2))
                planning_scene.addCube("righ_of_object", 0.5, (transx), (transy-0.5), (transz-0.2))
               
            state = "move_arm"
        
        elif(state == "move_arm"):
            '''
            moves arm around cube or in bin
            tries to implement joint interpgolated motion for a smoother grab
            '''
            moveX = transx
            moveY = transy
            moveZ = transz
            #move gripper
            gripperLength = 0.18
            if(holding_cube):
                gripper_over = Pose(Point((moveX), (moveY), (1)), Quaternion(0, 0.6, 0, 0.94))
                gripper_near = Pose(Point((moveX), (moveY), (0.7)), Quaternion(0, 0.6, 0, 0.94))
                
                gripper_poses=[gripper_over,gripper_near]
            
            elif(not holding_cube):    
                gripper_initial_pose = Pose(Point((moveX-gripperLength-0.15), (moveY-0.05), (moveZ)),Quaternion(0, 0, 0, 0))
                gripper_near_pose = Pose(Point((moveX-gripperLength-0.1), (moveY-0.05), (moveZ)),Quaternion(0, 0, 0, 0))
                #poses used for interpolated motion but do not work with the collision objects
                gripper_around = Pose(Point((moveX-gripperLength-0.02), (moveY), (moveZ)), Quaternion(0, 0, 0, 0))

                gripper_poses = [gripper_initial_pose, gripper_near_pose]            
            
            rospy.loginfo(gripper_poses)
            
            for pose in gripper_poses:
                # Finish building the Pose_stamped message
                # If the message stamp is not current it could be ignored
                gripper_pose_stamped.header.stamp = rospy.Time.now()
                # Set the message pose
                gripper_pose_stamped.pose = pose

                # Move gripper frame to the pose specified
                move_group.moveToPose(gripper_pose_stamped, gripper_frame)
                result = move_group.get_move_action().get_result()

                if result:
                    # Checking the MoveItErrorCode
                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Arm move success")
                    else:
                        rospy.logerr("Arm goal in state: %s",
                                     move_group.get_move_action().get_state())
                else:
                    rospy.logerr("MoveIt! failure no result returned.")
                rospy.loginfo("moving to object")
            grab_start_time = rospy.get_time()
            state = "move_around_object"
            
        elif(state == "move_around_object"):
            if(holding_cube):
                state = "grip"
            else:
                msg.linear.x = 0.1
                msg.angular.z = 0.1
                grab_time = now_time - grab_start_time
                rospy.loginfo(grab_time)
                if(grab_time > 2):
                    msg.linear.x = 0 
                    state = "grip" 
    
        elif(state == "grip"):
            '''
            either grip object or ungrip object
            '''
            msg.linear.x = 0
            if(not holding_cube):
                gripper_state = gripper_closed
            elif(holding_cube):
                gripper_state = gripper_open
            
            rospy.loginfo("Waiting for gripper_controller...")
            gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
            gripper_client.wait_for_server()
            rospy.loginfo("...connected.")

            gripper_goal = GripperCommandGoal()
            gripper_goal.command.max_effort = 10.0
            gripper_goal.command.position = gripper_state

            rospy.loginfo("Setting positions closed...")
            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result(rospy.Duration(5.0))

            time.sleep(1.0)
            rospy.loginfo("done")
            
            if(gripper_state == gripper_closed):
                holding_cube = True
                rospy.loginfo("Holding cube")
            elif(gripper_state == gripper_open):
                holding_cube = False            
                rospy.loginfo("Not holding cube")
            
            state = "delete_collisions"

        elif(state == "delete_collisions"):
            '''
            deletes all collision objects for next object located
            '''
            planning_scene.removeCollisionObject("my_front_ground")
            planning_scene.removeCollisionObject("my_back_ground")
            planning_scene.removeCollisionObject("my_right_ground")
            planning_scene.removeCollisionObject("my_left_ground")

            planning_scene.removeCollisionObject("under_object")
            planning_scene.removeCollisionObject("above_object")   
            planning_scene.removeCollisionObject("right_of_object")
            planning_scene.removeCollisionObject("under_right_of_object")
            planning_scene.removeCollisionObject("left_of_object")
            planning_scene.removeCollisionObject("under_left_of_object")
            planning_scene.removeCollisionObject("side_object_left")
            planning_scene.removeCollisionObject("side_object_right")
            reverse_start_time = rospy.get_time()   
            state = "reverse"

        elif(state == "reverse"):
            '''
            reverses to middle of room to prepare for spin
            reverses as no object avoidance implemented yet
            '''
            msg.linear.x = -0.5
            #realigning for drift in gazebo
            msg.angular.z = 0.15
            reverse_time = now_time - reverse_start_time
            if(reverse_time>2):
                msg.linear.x=0
                state = "restart"

        elif(state == "restart"):
            '''
            initialises robot pose and start of fsm timer
            '''
            InitialPose()
            start_time = rospy.get_time()
            state = "colourPick"

        pub.publish(msg)
        rate.sleep()

    
    rospy.loginfo("All cubes collected :)")
     


if __name__ == "__main__":
    
    try:
        turnRobot()
    except rospy.ROSInterruptException:
        pass
    





