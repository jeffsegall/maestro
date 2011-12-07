#!/usr/bin/python
from openravepy import *
from numpy import *
import time

try:
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('/opt/ros/diamondback/stacks/openrave_planning/openrave_robot_control/models/jaemi_hubo/jaemiHubo.robot.xml')

    while True:
	    continue

finally:
    env.Destroy()
