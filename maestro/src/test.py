#!/usr/bin/python
from openravepy import *
from numpy import *
import time
env = Environment()
env.SetViewer('qtcoin')
env.Load('openrave_robot_control/models/jaemi_hubo/jaemiHubo.robot.xml')
