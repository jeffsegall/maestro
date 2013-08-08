#!/usr/bin/env python

import time
from MaestroController import MaestroController

controller = MaestroController()
#controller.startMaestro()
#for i in range(0, 50000):
#controller.initRobot()
#controller.homeAll() 
#controller.enableAll()
#controller.initSensors()
controller.executeCommonStartUp()
controller.setJointPosition("RSP", -2)
controller.setJointVelocity("RSP", 2)
controller.get("RSP", "position")
print controller.get("RSP", "velocity")
controller.publishMessage("RSP", "position", 1.3, "")	
