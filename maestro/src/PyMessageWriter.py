#!/usr/bin/env python
'''
An object of this class should made by the user and then the user should be 
able to talk to maestro programaticly 
'''
import roslib; roslib.load_manifest('hubo_ros')
import rospy
import itertools
from hubomsg.msg import *

pyMessage = PythonMessage()

def sendJointCommand(name, position, velocity):
    commandList = []
    global the_HuboCommand
    #the_HuboCommand = HuboCommand() 
    the_HuboJointCommand = HuboJointCommand(name, position, velocity)
    commandList.append(the_HuboJointCommand)
    the_HuboCommand.joints = commandList
    the_HuboCommand.num_joints = 1

def sendMultiJointCommand(names, positions, velocities):
    commandList = []
    global the_HuboCommand
    #the_HuboCommand = HuboCommand()
    for name, position, velocity in itertools.izip(names, positions, velocities):
        commandList.append(HuboJointCommand(name, position, velocity))
    the_HuboCommand.joints = commandList
    the_HuboCommand.num_joints = len(commandList)

def testMessage():
   # rospy.init_node('message_writer', anonymous=True)
    pub = rospy.Publisher('Hubo/PythonCommand', PythonMessage)
   # while True:
    the_HuboJointCommand = HuboJointCommand("RSP", 1.5, 2)
    jointList = []
    jointList.append(the_HuboJointCommand)
    myHuboComm = PythonMessage(jointList, 1)
    pub.publish(myHuboComm)

def lookToPublish():
    
   
    while not rospy.is_shutdown():
        if the_HuboCommand.num_joints >=0:
            pub.publish(the_HuboCommand)  

if __name__ == '__main__':
    rospy.init_node('message_writer', disable_signals = False)
    try:
       
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
            
     
