#!/usr/bin/env python

import rospy
import time
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from armadillo_navigation_upgrade.srv import ser_message, ser_messageResponse
from std_msgs.msg import String
import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call
import moveit_commander

rospy.init_node('navigation_services')

def check_moveit_pose():
    robot = moveit_commander.RobotCommander()
    print("current arm state: ", robot.get_current_state())

def planning_cobra_center():
    #End#################################################################################################
    check_moveit_pose()
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    rospy.logerr('Planning to cobra-center!\n')
    time.sleep(1)
    print(BASE_DIR)
    proc = subprocess.Popen(["roslaunch " + BASE_DIR + "/robotican_demos_upgrade/launch/cobra_center.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)  
    while True:
        lin = proc.stdout.readline()
        if "success" in lin and "True" in lin:            
            break
        elif "success" in lin and "False" in lin:     
            break
        else:
            continue    
    proc.terminate()
    return

def _callback_navigate_corner_area(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(1.744, 4.711, 0)     # (2.935, 4.244, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.69)      # (0, 0, -1.554)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_open_area(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(3.553, -3.046, 0)   # (5.983, -2.680, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.680)    # (0, 0, -1.532)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    
    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)


def _callback_navigate_elevator(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(6.020, 4.257, 0)     # (5.8, 4.2, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 1.452 )      # (0, 0, 1.54)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    time.sleep(4)
    

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)


def _callback_navigate_room_1(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(7.212, 4.388, 0)    # (-13.759, -3.9, 0) (5.762 ,4.326, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)     # (0, 0, 3.111) (0, 0, 1.39)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    time.sleep(4)
    

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)


def _callback_navigate_room_peson(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(7.212, 4.388, 0)    # (-6.527, -6.922, 0)  (5.762 ,4.326, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)      # (0, 0, -1.361) (0, 0, 1.39)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)


def _callback_navigate_room_place(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(7.212, 4.388, 0)        # (-13.34, -3.733, 0) (5.762 ,4.326, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)     # (0, 0, -1.552) (0, 0, 1.39)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_auditorium(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(9.102, -1.511, 0)     # (10.485, -1.6, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.696)      # (0, 0, -1.595)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_lab_211(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up

    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")

    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(-0.449, 1.429, 0)     # (1.816, 0.158, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -0.076)      # (0, 0, 0.155)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_outside_lab211(req):
    
    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(-5.904, 4.274, 0)     # (-4.284, 2.952, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 3.093)      # (0, 0, -3.0)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    time.sleep(2)
    #repeat just for solid execution
    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(-6.609, 4.020, 0)     # (-4.84, 2.952, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 3.093)      # (0, 0, -3.0)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_corridor(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(8.140, 3.194, 0)     # (8.506, 3.696, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)      # (0, 0, -0.037)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_table(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(8.12, -1.8, 0)      # 8.1 -1.760 
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.694)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)


def _callback_navigate_person(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(3.750, 0.322, 0) #8.050
    orientation = tf.transformations.quaternion_from_euler(0, 0, -2.903)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_pour(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(10.408 ,3.604, 0) # Point(7.752 ,3.231, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -0.085)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

def _callback_navigate_place(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(9.995, 3.6, 0) #Point(7.794 ,3.228, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.71)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the open area")
        ser_messageResponse(True)
        msg = String()
        msg.data = "success"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)
    else:
        rospy.logerr("The robot failed to reach the open area")
        ser_messageResponse(False)
        msg = String()
        msg.data = "failure"
        for ii in range(7):
            feedback_pub.publish(msg)
            time.sleep(1)

#it must be in cobra-center position before starting navigation
planning_cobra_center()

rospy.Service("/elevator_go", ser_message, _callback_navigate_elevator)
rospy.loginfo('"/elevator_go" is waiting for request...')

rospy.Service("/room_1_go", ser_message, _callback_navigate_room_1)
rospy.loginfo('"/elevator_go" is waiting for request...')

rospy.Service("/room_person_go", ser_message, _callback_navigate_room_peson)
rospy.loginfo('"/elevator_go" is waiting for request...')

rospy.Service("/room_place_go", ser_message, _callback_navigate_room_place)
rospy.loginfo('"/elevator_go" is waiting for request...')

rospy.Service("/auditorium_go", ser_message, _callback_navigate_auditorium)
rospy.loginfo('"/auditorium_go" is waiting for request...')

rospy.Service("/lab_211_go", ser_message, _callback_navigate_lab_211)
rospy.loginfo('"/lab_211_go" is waiting for request...')

rospy.Service("/corridor_go", ser_message, _callback_navigate_corridor)
rospy.loginfo('"/corridor_go" is waiting for request...')

rospy.Service("/outside_lab_211_go", ser_message, _callback_navigate_outside_lab211)
rospy.loginfo('"/outside_lab_211_go" is waiting for request...')

rospy.Service("/open_area", ser_message, _callback_navigate_open_area)
rospy.loginfo('"/open_area" is waiting for request...')

rospy.Service("/corner_area", ser_message, _callback_navigate_corner_area)
rospy.loginfo('"/corner_area" is waiting for request...')

rospy.Service("/table_go", ser_message, _callback_navigate_table)
rospy.loginfo('"/table_go" is waiting for request...')

rospy.Service("/person_go", ser_message, _callback_navigate_person)
rospy.loginfo('"/person_go" is waiting for request...')

rospy.Service("/pour_nav_go", ser_message, _callback_navigate_pour)
rospy.loginfo('"/pour_nav_go" is waiting for request...')

rospy.Service("/place_nav_go", ser_message, _callback_navigate_place)
rospy.loginfo('"/place_nav_go" is waiting for request...')

rospy.loginfo("all of the navigation services are waiting for request...")
rospy.spin()

#Command order senario 1:
# table_go -> pick_go -> pour_nav_go -> pour_go -> place_nav_go -> place_go_table -> pour_nav_go -> pick_go_other ->person_go -> place_go_person -> free_can 
