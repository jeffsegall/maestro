#!/usr/bin/python
import roslib; roslib.load_manifest('openrave_robot_control')
roslib.load_manifest('maestro')
from openravepy import *
import rospy
from numpy import *
from hubomsg.msg import HuboCmd
from std_msgs.msg import String
import sys

__author__="Christopher T. Cannon<cannon@drexel.edu>"
__date__ ="$Dec 8, 2011 1:22:15 PM$"

class Maestro:
    def __init__(self):
        print sys.path
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.Load('/opt/ros/diamondback/stacks/openrave_planning/' +
        'openrave_robot_control/models/jaemi_hubo/jaemiHubo.robot.xml')
        physics = RaveCreatePhysicsEngine(self.env,'ode')
        physics.SetGravity(numpy.array((0,0,0)))
        #self.env.SetPhysicsEngine(physics)
        self.robot = self.env.GetRobots()[0]
        self.robot.GetLinks()[0].SetStatic(True)
        self.env.StopSimulation()
        self.env.StartSimulation(timestep=0.001)
        rospy.init_node('maestro', anonymous=False)
        rospy.Subscriber('/hubo_cmd', HuboCmd, self.callback)
        rospy.spin()

    def callback(self,data):
        print "Got callback. Joint, Angle = ", data.joint,  ", ",  data.angle
        #message = data.data.split(',')
        #if message[0] == "head_yaw":
        #    self.set_head_yaw(int(message[1]))
	self.robot.SetJointValues([data.angle],[data.joint])	


    def set_head_yaw(self, value):
        self.robot.SetJointValues([value],[1])

    def set_torso_yaw(self, value):
        self.robot.SetJointValues([value],[0])

    def set_left_shoulder_roll(self,value):
        self.robot.SetJointValues([value],[3])

    def move_left_shoulder_pitch(self,value):
        self.robot.SetJointValues([value],[4])

    def move_left_elbow_roll(self,value):
        self.robot.SetJointValues([value],[5])

    def move_left_elbow_yaw(self,value):
        self.robot.SetJointValues([value],[6])

    def move_left_wrist_roll(self,value):
        self.robot.SetJointValues([value],[7])

    def move_right_shoulder_roll(self,value):
        self.robot.SetJointValues([value],[11])

    def move_right_shoulder_pitch(self,value):
        self.robot.SetJointValues([value][12])

    def move_right_elbow_roll(self,value):
        self.robot.SetJointValues([value],[13])

    def move_right_elbow_yaw(self,value):
        self.robot.SetJointValues([value],[14])

    def move_right_wrist_roll(self,value):
        self.robot.SetJointValues([value],[15])

    def move_left_hip_yaw(self,value):
        self.robot.SetJointValues([value],[19])

    def move_left_hip_lateral(self,value):
        self.robot.SetJointValues([value],[20])

        
    def start(self):
        joint = 20
        while True:
            continue


if __name__ == "__main__":
    m = Maestro()
    m.start()
