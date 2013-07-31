#!/usr/bin/env python

import time
from MaestroController import MaestroController

controller = MaestroController()
#for i in range(0, 50000):
controller.initRobot()
time.sleep(.01)
controller.homeAll() 
time.sleep(.01)
controller.enableAll()
time.sleep(.01)
controller.test()
