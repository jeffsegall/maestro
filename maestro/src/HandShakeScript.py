#!/usr/bin/env python

import time
from MaestroController import MaestroController

controller = MaestroController()
controller.executeCommonStartUp()
print("Starting up")
time.sleep(15)
print("Giving Command")
controller.setJointPosition("REP", "-1.2")
controller.waitForJoint("REP")
controller.publishMessage("RSP REP", "position position", "-1.2 0", "")
