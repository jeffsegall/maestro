#!/usr/bin/env python

import time
from MaestroController import MaestroController

controller = MaestroController()
controller.executeCommonStartUp()
print("Starting up")
print("Giving Command")
controller.setJointPosition("REP", "-1.2")
#controller.doWhen(10, "LSP", "position", "-1.0", "")
controller.waitForJoint("REP")
controller.publishMessage("RSP REP", "position position", "-1.2 0", "")
