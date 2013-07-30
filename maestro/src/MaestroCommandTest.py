#!/usr/bin/env python

import time
from MaestroController import MaestroController

controller = MaestroController()
for i in range(0, 50000): 
 time.sleep(.01)
 controller.test()
