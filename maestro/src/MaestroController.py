#!/usr/bin/env python

import roslib; roslib.load_manifest('maestro')
import rospy
import time
import subprocess
from hubomsg.msg import *

class MaestroController:
	def __init__(self):
		print "Init"
		self.newVal = False
		self.value = 0.0
		#self.startMaestro()
		rospy.init_node("Maestro_Commands")
		self.pub = rospy.Publisher('Maestro/Control', PythonMessage)
		rospy.Subscriber("Maestro/Message", MaestroMessage, self.display)
		rospy.sleep(2)
	def startMaestro(self):
		subprocess.call([""])
	def display(self, message):
		self.newVal = True
		self.value = message.value
	def test(self):
		pyMessage = PythonMessage("RSP", "position", 1.5, "")
		self.pub.publish(pyMessage)
		print "Published a message"
		time.sleep(.01)
	def setJointPosition(self, joint, value):
		pyMessage = PythonMessage(joint, "position", value, "")
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def setJointVelocity(self, joint, value):
		pyMessage = PythonMessage(joint, "velocity", value, "")
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def enableAll(self):
		pyMessage = PythonMessage("", "EnableAll", 0, "")
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def homeAll(self):
		pyMessage = PythonMessage("", "HomeAll", 0, "")
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def initRobot(self):
		pyMessage = PythonMessage("", "initRobot", 0, "")
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def initSensors(self):
		pyMessage = PythonMessage("", "InitializeSensors", 0, "")
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def publishMessage(self, joint, command, value, target):
		pyMessage = PythonMessage(joint, command, value, target)
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def executeCommonStartUp(self):
		self.initRobot()
		self.homeAll()
		self.enableAll()
		self.initSensors()
	def home(self, target):
		pyMessage = PythonMessage("", "Home", 0, target)
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def enable(self, target):
		pyMessage = PythonMessage("", "Enable", 0, target)
		self.pub.publish(pyMessage)
		time.sleep(.01)
	def get(self, joint, target):
		pyMessage = PythonMessage(joint, "Get", 0, target)
		self.pub.publish(pyMessage)
		time.sleep(.01)
		if self.newVal:
			self.newVal = False
			return self.value
