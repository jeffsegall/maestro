#!/usr/bin/python
import roslib; roslib.load_manifest('openrave_robot_control')
import rospy
from openravepy import *
from numpy import *
import time
from std_msgs.msg import String

def callback(data):
    print data.data

def listener():
    rospy.init_node('hubo_openrave', anonymous=False)
    rospy.Subscriber('/string_out', String, callback)
    rospy.spin()

env = Environment()
env.SetViewer('qtcoin')
env.Load('/opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/models/jaemi_hubo/jaemiHubo.robot.xml')

robot = env.GetRobots()[0]

listener()
