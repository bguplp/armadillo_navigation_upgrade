#!/usr/bin/env python 

import rospy
import rosnode
from actionlib_msgs.msg import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
import tf.transformations
import tf2_ros
from sensor_msgs.msg import LaserScan
import numpy as np
from armadillo_navigation_upgrade.srv import exit_elevator, exit_elevatorResponse
from std_msgs.msg import String
from armadillo_navigation_upgrade.srv import switch_map, switch_mapResponse

global z, error_max, num, node_name
num = -10

def get_scan(msg):
    global z
    z = np.array(msg.ranges)     

def wrong_floor_num_func():
    rospy.logerr('Invalid floor number!\nplease insert "0" for first floor and "1" for second floor')
    new_floor_num = input('\nwaiting for your respond choose <"0","1">: ')
    print new_floor_num," ", type(new_floor_num), " new_floor_num!!!!!!!!!!!!!!!!!!!"
    return int(new_floor_num), False

def first_floor_func():
    return "/home/armadillo/catkin_ws/src/armadillo_navigation_upgrade/map/simulation_map_first_floor.yaml" ,True

def second_floor_func():
    return "/home/armadillo/catkin_ws/src/armadillo_navigation_upgrade/map/simulation_map_elevator.yaml" ,True

def switch_func(floor_num, flag = True):
    global new_map_path
    print floor_num," ", type(floor_num), " floor_num!!!!!!!!!!!!!!!!!!!"
    switcher = {
        0: first_floor_func,
        1: second_floor_func
    } 
    switch_to = switcher.get(floor_num, wrong_floor_num_func)
    new_floor, flag = switch_to()
    print flag," ", type(flag), " flag!!!!!!!!!!!!!!!!!!!"
    if not flag:
        switch_func(floor_num = new_floor)
    else:
        new_map_path = new_floor

def exit_elevator_func(req):
    global z, error_max, num, new_map_path

    rospy.Subscriber('/scan', LaserScan, get_scan)

    # input_pose = PoseStamped()
    # input_pose.header.stamp = rospy.Time.now()
    # input_pose.header.frame_id = 'map'
    # input_pose.pose.position.x = 6.5 
    # input_pose.pose.position.y = 6.3
    # input_pose.pose.position.z = 0
    # #input_pose.pose.orientation.x = 
    # #input_pose.pose.orientation.y =
    # #input_pose.pose.orientation.z =
    # #input_pose.pose.orientation.w =

    # #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    # #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    # #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    # input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,-1.57))

    # #define a client for to send goal requests to the move_base server through a SimpleActionClient
    # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # #wait for the action server to come up
    # while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
    #     rospy.loginfo("Waiting for the move_base action server to come up")
    # '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
    #     rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    # goal = MoveBaseGoal()
    # #set up the frame parameters
    # goal.target_pose.header.frame_id = "/map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # # moving towards the goal*/
    # goal.target_pose = input_pose

    # rospy.loginfo("Sending goal location ...")
    # ac.send_goal(goal)	
    # ac.wait_for_result(rospy.Duration(60))

    # if(ac.get_state() ==  GoalStatus.SUCCEEDED):
    #     rospy.loginfo("You have reached initial position for elevator entrance")
        # exit_elevatorResponse(True)
    
    # else:
    #     rospy.loginfo("The robot failed to the initial position for elevator entrance")
        # exit_elevatorResponse(False)


    z = np.array(rospy.wait_for_message("/scan", LaserScan).ranges)
    
    # rospy.sleep(0.5)
    temp_z = z
    error = np.absolute(z - temp_z)
    error[~np.isfinite(error)] = 0
    error_max = np.sort(error)[num]
    # flag = True
    while (error_max < 0.15):
        error = np.absolute(z - temp_z)
        temp_z = z
        error[~np.isfinite(error)] = 0
        error_max = np.sort(error)[num]

        rospy.logwarn("node \"/inside_elevator\" is alive")                

        #rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
        rospy.sleep(0.5)
        # if (flag):
        #     flag = False
        #     error_max = 0
        #     rospy.sleep(1)
    rospy.logwarn("waiting for switch map service...")
    rospy.wait_for_service('/switch_map')
    switch_srv = rospy.ServiceProxy('/switch_map', switch_map)
    switch_func(floor_num = req.floor_num)
    print new_map_path, " new_map_path!!!!!!!!!!!!!!" 
    switch_srv(new_map_path)

    success, fail = rosnode.kill_nodes([node_name.data])
    rospy.loginfo("success = "+ str(success)+ "fail = "+ str(fail))
    if fail != []:
        rospy.logerr("node \"/inside_elevator\" is still alive!!")
    elif success != []:
        rospy.loginfo("node \"/inside_elevator\" died!!")

    rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = 6.625 
    input_pose.pose.position.y =3.441
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,-1.57))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You got out of the elevator")
        # exit_elevatorResponse(True)

    else:
        rospy.loginfo("The robot failed to get out of the elevator")
        # exit_elevatorResponse(False)


rospy.init_node('exit_elevator', anonymous=True)
rospy.Service("/exit_elevator", exit_elevator, exit_elevator_func)
node_name = String()
node_name = rospy.wait_for_message("/inside_elevator_node_name", String)
rospy.loginfo("exit elevator service is waiting for request...")

rospy.spin()