#!/usr/bin/env python
import roslib; roslib.load_manifest('openrave_robot_control')
from openravepy import *
import rospy
from numpy import *
from std_msgs.msg import String
import time

__author__="Christopher T. Cannon<cannon@drexel.edu>"
__date__ ="$Dec 8, 2011 1:22:15 PM$"

class Maestro:
    def __init__(self):
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.Load('/opt/ros/diamondback/stacks/openrave_planning/' +
        'openrave_robot_control/models/jaemi_hubo/jaemiHubo.robot.xml')
        #physics = RaveCreatePhysicsEngine(self.env,'ode')
        #physics.SetGravity(numpy.array((0,0,0)))
        #self.env.SetPhysicsEngine(physics)
        self.robot = self.env.GetRobots()[0]
        self.robot.GetLinks()[0].SetStatic(True)
        self.env.StopSimulation()
        self.env.StartSimulation(timestep=0.001)
        rospy.init_node('maestro', anonymous=False)
        rospy.Subscriber('/hubo_cmd', String, self.callback)
        self.pub = rospy.Publisher('/ros_in', String)

    def __del__(self):
        self.env.Destroy()

    def callback(self,data):
        with self.robot:
            self.robot.SetJointValues([data.angle],[data.joint])
            if self.robot.CheckSelfCollision():
                send_error_message(data.joint,data.angle,"Self Collision.")
                return None
            elif self.env.CheckCollision(self.robot):
                send_error_message(data.joint,data.angle,"Environment Collision.")
                return None
        
        self.robot.SetJointValues([data.angle],[data.joint])

    def send_error_message(self,joint,angle,message):
        self.pub.publish(joint,angle,message)

    def set_torso_yaw(self, value):
        self.robot.SetJointValues([value],[0])

    def set_head_yaw(self, value):
        self.robot.SetJointValues([value],[1])

    def set_left_shoulder_roll(self,value):
        self.robot.SetJointValues([value],[3])

    def set_left_shoulder_pitch(self,value):
        self.robot.SetJointValues([value],[4])

    def set_left_elbow_roll(self,value):
        self.robot.SetJointValues([value],[5])

    def set_left_elbow_yaw(self,value):
        self.robot.SetJointValues([value],[6])

    def set_left_wrist_roll(self,value):
        self.robot.SetJointValues([value],[7])

    def set_right_shoulder_roll(self,value):
        self.robot.SetJointValues([value],[11])

    def set_right_shoulder_pitch(self,value):
        self.robot.SetJointValues([value][12])

    def set_right_elbow_roll(self,value):
        self.robot.SetJointValues([value],[13])

    def set_right_elbow_yaw(self,value):
        self.robot.SetJointValues([value],[14])

    def set_right_wrist_roll(self,value):
        self.robot.SetJointValues([value],[15])

    def set_left_hip_yaw(self,value):
        self.robot.SetJointValues([value],[19])

    def set_left_hip_roll(self,value):
        self.robot.SetJointValues([value],[20])

    def set_left_hip_pitch(self,value):
        self.robot.SetJointValues([value],[21])

    def set_letf_knee_pitch(self,value):
        self.robot.SetJointValues([value],[22])

    def set_left_foot_pitch(self,value):
        self.robot.SetJointValues([value],[23])

    def set_left_foot_roll(self,value):
        self.robot.SetJointValues([value],[24])

    def set_right_hip_yaw(self,value):
        self.robot.SetJointValues([value],[26])

    def set_right_hip_roll(self,value):
        self.robot.SetJointValues([value],[27])

    def set_right_hip_pitch(self,value):
        self.robot.SetJointValues([value],[28])

    def set_right_knee_pitch(self,value):
        self.robot.SetJointValues([value],[29])

    def set_right_foot_pitch(self,value):
        self.robot.SetJointValues([value],[30])

    def set_right_foot_roll(self,value):
        self.robot.SetJointValue([value],[31])

    def start(self):
        while True:
            continue


if __name__ == "__main__":
    m = Maestro()
    m.start()
