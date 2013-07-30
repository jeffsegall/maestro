#!/usr/bin/env python

import roslib; roslib.load_manifest('maestro')
import rospy
from hubomsg.msg import *

if __name__ == '__main__':
	rospy.init_node("Commands_for_hubo");
	pub = rospy.Publisher('Maestro/Control', PythonMessage)
	while not rospy.is_shutdown():
		rospy.sleep(10)
		#rospy.init_node("Commands_for_hubo");
		huboJointCommand1 = HuboJointCommand("RSP", 1.5, 2)
		jointList = []
		jointList.append(huboJointCommand1)
		pythonMessage1 = PythonMessage(jointList, 1)
		#pub = rospy.Publisher('Maestro/Control', PythonMessage)
		pub.publish(pythonMessage1)
		
