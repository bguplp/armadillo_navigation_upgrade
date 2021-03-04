#!/usr/bin/env python

import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from armadillo_navigation_upgrade.srv import move_to_point, move_to_pointResponse
from std_srvs.srv import Trigger
from std_msgs.msg import Float64


def planning_cobra_center(pose="cobra_center_pose"):
    try:
        rospy.wait_for_service(pose)
        cobra_center_proxy = rospy.ServiceProxy(pose, Trigger)
        resp = cobra_center_proxy()
        return resp.success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_toroso(hight=0.01):
    pub_torso_cmd = rospy.Publisher("/torso_effort_controller/command", Float64, queue_size=1)
    pub_torso_cmd.publish(Float64(hight))
    for __ in range(3):
        pub_torso_cmd.publish(Float64(hight))
        rospy.sleep(0.01)
    rospy.sleep(1)


def _callback_nav_service(req, x, y, yaw):
    #   it must be in cobra-center position before starting navigation
    planning_cobra_center()
    set_toroso()
    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    # set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position = Point(x, y, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached goal")
        return move_to_pointResponse("true")
    else:
        rospy.logerr("The robot failed to reach goal")
        return move_to_pointResponse("false")


def init_nav_service(service_name, x, y, yaw):
    # method_name = "_callback_"+ service_name
    # possibles = globals().copy()
    # possibles.update(locals())
    # callback = possibles.get(method_name)
    # service = "/"+service_name
    rospy.Service(str("/"+service_name), move_to_point, lambda req: _callback_nav_service(req, x, y, yaw))


def service_nav():
    services = rospy.get_param("/nav_services").keys()
    for ii in range(len(services)):
        service_name = services[ii]
        nav_goal = rospy.get_param("/nav_services/"+service_name)
        x, y, yaw = nav_goal['x'], nav_goal['y'], nav_goal['yaw']
        init_nav_service(service_name, x, y, yaw)


def _callback_waypoint_nav(req, x, y, yaw, service_name):
    #   it must be in cobra-center position before starting navigation
    planning_cobra_center()
    set_toroso()
    waypoint = rospy.get_param("/nav_waypoint_services/" + service_name + "/waypoint_list")
    for ii in range(len(waypoint)):
        # define a client to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.logwarn("Waiting for the move_base action server to come up")
        '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
        goal = MoveBaseGoal()
        # set up the frame parameters
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # moving towards the goal*/
        goal.target_pose.pose.position = Point(x[ii], y[ii], 0)
        orientation = tf.transformations.quaternion_from_euler(0, 0, yaw[ii])
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)
        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("Reached waypoint "+waypoint[ii])
            return move_to_pointResponse("true")
        else:
            rospy.logwarn("Failed reaching waypoint "+waypoint[ii])
            return move_to_pointResponse("false")


def init_waypoint_service(service_name, x, y, yaw):
    rospy.Service(str("/"+service_name), move_to_point, lambda req: _callback_waypoint_nav(req, x, y, yaw, service_name))


def waypoint_nav():
    services = rospy.get_param("/nav_waypoint_services")
    services = rospy.get_param("/nav_waypoint_services").keys()
    for ii in range(len(services)):
        X = []
        Y = []
        Yaw = []
        service_name = services[ii]
        waypoint_list = rospy.get_param("/nav_waypoint_services/" + service_name + "/waypoint_list")
        for jj in range(len(waypoint_list)):
            waypoint_name = waypoint_list[jj]
            waypoint_goal = rospy.get_param("/nav_waypoint_services/" + service_name + "/" + waypoint_name)
            x, y, yaw = waypoint_goal['x'], waypoint_goal['y'], waypoint_goal['yaw']
            X.append(x)
            Y.append(y)
            Yaw.append(yaw)
        init_waypoint_service(service_name, X, Y, Yaw)


if __name__ == "__main__":
    rospy.init_node('navigation_service_node')
    waypoint_nav()
    service_nav()

    rospy.loginfo("navigation service node is waiting for request...")
    while not rospy.is_shutdown():
        rospy.sleep(1)
    rospy.spin()

# Command order senario 1:
# table_go -> pick_go -> pour_nav_go -> pour_go -> place_nav_go -> place_go_table -> pour_nav_go -> pick_go_other ->person_go -> place_go_person -> free_can
