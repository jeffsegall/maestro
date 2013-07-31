#!/usr/bin/env python

import roslib; roslib.load_manifest('maestro')
import rospy
from hubomsg.msg import *

class MaestroController:
	def __init__(self):
		print "Init"
		rospy.init_node("Maestro_Commands")
		self.pub = rospy.Publisher('Maestro/Control', PythonMessage)
		rospy.sleep(2)
	def test(self):
		pyMessage = PythonMessage("RSP", "position", 1.5)
		self.pub.publish(pyMessage)
		print "Published a message"
	def enableAll(self):
		pyMessage = PythonMessage("", "EnableAll", 0)
		self.pub.publish(pyMessage)
	def homeAll(self):
		pyMessage = PythonMessage("", "HomeAll", 0)
		self.pub.publish(pyMessage)
	def initRobot(self):
		pyMessage = PythonMessage("", "initRobot", 0)
		self.pub.publish(pyMessage)
