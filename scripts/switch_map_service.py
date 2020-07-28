#!/usr/bin/env python 

import rospy
import rosnode
import subprocess, shlex
from subprocess import Popen, PIPE, call
from armadillo_navigation_upgrade.srv import switch_map, switch_mapResponse

def switch_map_func(req):
    success, fail = rosnode.kill_nodes(["/map_server"])
    rospy.loginfo("success = "+ str(success)+ "fail = "+ str(fail))
    if fail != []:
        rospy.logwarn("node \"/map_server\" is still alive")
    elif success != []:
        rospy.loginfo("node \"/map_server\" died")

    command_line = "roslaunch armadillo_navigation_upgrade map_server.launch map:="+req.map_name
    args = shlex.split(command_line)
    p = subprocess.Popen(args, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    rospy.loginfo("a new map service node have been launched, with new map!") 
    rospy.sleep(2)
    p.terminate()

rospy.init_node("switch_map_service", anonymous=True, disable_signals=True)
rospy.Service("/switch_map", switch_map, switch_map_func)
rospy.spin()