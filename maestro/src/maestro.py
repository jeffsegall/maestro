#!/usr/bin/env python
"""MAESTRO

A python script that creates a simulation server in openRAVE which
communicates over ROS.

usage: python maestro.py [options]

Options:
    -h, --help                  Show this help
    -r ..., --robot=...         Robot XML (COLLADA format)
    -s ..., --subscriber=...    ROS topic subscription
    -p ..., --publisher=...     ROS topic publication
    -g ..., --gravity=...       OpenRAVE Physics Engine (True/False)
"""
__author__="Christopher T. Cannon<cannon@drexel.edu>"
__date__ ="$Dec 8, 2011 1:22:15 PM$"

import roslib; roslib.load_manifest('openrave_robot_control')
roslib.load_manifest('maestro')
from openravepy import *
import rospy
from numpy import *
from hubomsg.msg import HuboCmd
from std_msgs.msg import String
import sys
import getopt

class Maestro:
    def __init__(self,robot_xml,subscriber,publisher,gravity):
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.Load(robot_xml)

        if gravity:
            physics = RaveCreatePhysicsEngine(self.env,'ode')
            physics.SetGravity(numpy.array((0,0,-9.8)))
            self.env.SetPhysicsEngine(physics)

        self.robot = self.env.GetRobots()[0]
        self.robot.GetLinks()[0].SetStatic(True)
        self.env.StopSimulation()
        self.env.StartSimulation(timestep=0.001)
        rospy.init_node('maestro', anonymous=False)
        rospy.Subscriber(subscriber, HuboCmd, self.callback)
        self.pub = rospy.Publisher(publisher, HuboCmd)

    def __del__(self):
        self.env.Destroy()

    def callback(self,data):
        with self.robot:
            self.robot.SetJointValues([data.angle],[data.joint])
            if self.robot.CheckSelfCollision():
                self.send_error_message(data.joint,data.angle,"Self Collision.")
                return None
            elif self.env.CheckCollision(self.robot):
                send_error_message(data.joint,data.angle,"Environment Collision.")
                return None
        
        self.robot.SetJointValues([data.angle],[data.joint])

    def send_error_message(self,joint,angle,message):
        msg = HuboCmd()
	msg.joint = joint
	msg.angle = angle
	msg.msg = message
        self.pub.publish(msg)

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

    def set_left_wrist_pitch(self,value):
        self.robot.SetJointValues([value],[8])

    def set_left_wrist_yaw(self,value):
        self.robot.SetJointValues([value],[9])

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

    def set_right_wrist_pitch(self,value):
        self.robot.SetJointValues([value],[16])

    def set_right_wrist_yaw(self,value):
        self.robot.SetJointValues([value],[17])

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
        while not rospy.is_shutdown():
            continue

def usage():
    """Prints the usage."""
    print __doc__

def main(argv):
    """Parses the command line arguements and starts the program."""
    global robotXml,subscriber,publisher,gravity

    try:
        opts,args = getopt.getopt(argv, "hr:s:p:g:", ["help","robot=","subscriber=",
        "publisher=","gravity="])
    except getopt.GetoptError:
        usage()
        sys.exit(2)
    for opt,arg in opts:
        if opt in ("-h","--help"):
            usage()
            sys.exit()
        elif opt in ("-r","--robot"):
            robotXml = arg
        elif opt in ("-s","--subscriber"):
            subscriber = arg
        elif opt in ("-p","--publisher"):
            publisher = arg
        elif opt in ("-g","--gravity"):
            if arg.lower() == "true":
                gravity = True
            else:
                gravity = False

    Maestro(robotXml,subscriber,publisher,gravity).start()

if __name__ == "__main__":
    main(sys.argv[1:])
