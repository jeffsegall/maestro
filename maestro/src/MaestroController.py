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
	#	rospy.spinOnce()
	def test(self):
		#rospy.init_node("Maestro_Commands")
		#pub = rospy.Publisher('Maestro/Control', PythonMessage)
		huboJoint = HuboJointCommand("RSP", 1.5, 2)
		jointList = []
		jointList.append(huboJoint)
		pyMessage = PythonMessage(jointList, 1);
	#	rospy.sleep(.2578125)
		self.pub.publish(pyMessage)
		print "Published a message"

