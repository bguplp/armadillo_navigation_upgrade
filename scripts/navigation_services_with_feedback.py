#!/usr/bin/env python

import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from armadillo_navigation_upgrade.srv import move_to_point, move_to_pointResponse


import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

def planning_cobra_center():
    ######  End  #######
    rospy.logwarn('Planning to cobra-center! please wait for navigation service node to be ready.\n')
    time.sleep(1)
    proc = subprocess.Popen(["roslaunch robotican_demos_upgrade cobra_center.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)  
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

def _callback_nav_service(req, x, y, yaw):
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
    goal.target_pose.pose.position =  Point(x ,y, 0) 
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached goal")
        move_to_pointResponse("true")
    else:
        rospy.logerr("The robot failed to reach goal")
        move_to_pointResponse("false")

def init_nav_service(service_name, x, y, yaw):
    # method_name = "_callback_"+service_name 
    # possibles = globals().copy()
    # possibles.update(locals())
    # callback = possibles.get(method_name)
    # service = "/"+service_name
    rospy.Service(str("/"+service_name), move_to_point, lambda req: _callback_nav_service(req, x, y, yaw))

def service_nav():
    services = rospy.get_param("/nav_services").keys() 
    for ii in range (len(services)):
        service_name = services[ii]
        nav_goal = rospy.get_param("/nav_services/"+service_name)
        x, y, yaw = nav_goal['x'], nav_goal['y'], nav_goal['yaw']
        init_nav_service(service_name, x, y, yaw)

def _callback_waypoint_nav(req, x, y, yaw, service_name):
    waypoint = rospy.get_param("/nav_waypoint_services/"+service_name+"/waypoint_list")
    for ii in range (len(waypoint)):
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
        goal.target_pose.pose.position =  Point(x[ii] ,y[ii], 0)
        orientation = tf.transformations.quaternion_from_euler(0, 0, yaw[ii])
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)	
        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("Reached waypoint "+waypoint[ii])
            move_to_pointResponse("true")
        else:
            rospy.logwarn("Failed reaching waypoint "+waypoint[ii])
            move_to_pointResponse("false")

def init_waypoint_service(service_name, x, y, yaw):
    rospy.Service(str("/"+service_name), move_to_point, lambda req: _callback_waypoint_nav(req, x, y, yaw, service_name))

def waypoint_nav():
    services = rospy.get_param("/nav_waypoint_services")
    services = rospy.get_param("/nav_waypoint_services").keys() 
    for ii in range (len(services)):
        X = []; Y = []; Yaw = []
        service_name = services[ii]
        waypoint_list = rospy.get_param("/nav_waypoint_services/"+service_name+"/waypoint_list")
        for jj in range(len(waypoint_list)):
            waypoint_name = waypoint_list[jj]
            waypoint_goal = rospy.get_param("/nav_waypoint_services/"+service_name+"/"+waypoint_name)
            x, y, yaw = waypoint_goal['x'], waypoint_goal['y'], waypoint_goal['yaw']
            X.append(x)
            Y.append(y)
            Yaw.append(yaw)
        init_waypoint_service(service_name, X, Y, Yaw)

if __name__ == "__main__":
    rospy.init_node('navigation_service_node')
    #   it must be in cobra-center position before starting navigation
    planning_cobra_center()
    waypoint_nav()
    service_nav()

    rospy.loginfo("navigation service node is waiting for request...")
    rospy.spin()

#Command order senario 1:
# table_go -> pick_go -> pour_nav_go -> pour_go -> place_nav_go -> place_go_table -> pour_nav_go -> pick_go_other ->person_go -> place_go_person -> free_can 
