#!/usr/bin/python
from openravepy import *
import numpy
import time

__author__="Christopher T. Cannon<cannon@drexel.edu>"
__date__ ="$Dec 8, 2011 1:22:15 PM$"

class Maestro:
    def __init__(self):
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.Load('/opt/ros/diamondback/stacks/openrave_planning/' +
        'openrave_robot_control/models/jaemi_hubo/jaemiHubo.robot.xml')
        physics = RaveCreatePhysicsEngine(self.env,'ode')
        physics.SetGravity(numpy.array((0,0,-9.8)))
        self.env.SetPhysicsEngine(physics)


    def start(self):
        with self.env:
            robot = self.env.GetRobots()[0]
            robot.GetLinks()[0].SetStatic(True)
            self.env.StopSimulation()
            self.env.StartSimulation(timestep=0.001)

            robot.GetController().SetDesired(0)
            robot.WaitForController()
        while True:
            continue

if __name__ == "__main__":
    m = Maestro()
    m.start()
