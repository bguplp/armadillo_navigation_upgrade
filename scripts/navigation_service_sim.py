#!/usr/bin/env python

import rospy
import time
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from armadillo_navigation_upgrade.srv import navigate_srv, navigate_srvResponse
from std_msgs.msg import String
import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call
import moveit_commander

rospy.init_node('navigation_service')

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
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    return

def _callback_navigate_goal(req):
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
    goal.target_pose.pose.position =  req.goal     # (2.935, 4.244, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, req.yaw)      # (0, 0, -1.554)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location (waiting no more than 60 seconds)...")
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))
    

    feedback_pub = rospy.Publisher("feedback_for_nav",String,  queue_size=1)
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.logerr("You have reached the desired location")
        return navigate_srvResponse(True)
    else:
        rospy.logerr("The robot failed to reach the desired location")
        return navigate_srvResponse(False)



#it must be in cobra-center position before starting navigation
# planning_cobra_center()
 
rospy.Service("/custom_actions/navigate", navigate_srv, _callback_navigate_goal)
rospy.loginfo('"/custom_actions/navigate" is waiting for request...')

rospy.spin()

#Command order senario 1:
# table_go -> pick_go -> pour_nav_go -> pour_go -> place_nav_go -> place_go_table -> pour_nav_go -> pick_go_other ->person_go -> place_go_person -> free_can 
