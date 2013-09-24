#!/usr/bin/env python

import roslib; roslib.load_manifest('hubomsg')
import rospy
import time
import sys
import subprocess
import wave
import audioop
import struct
from audio_common_msgs.msg import AudioData
from hubomsg.msg import *


def update(AudioData):
	try:
 		#avg = audioop.avg(AudioData.data, 2)
		fms = str(len(AudioData.data)/2) + "H"
		#print fms
		newData = (struct.unpack(fms, AudioData.data))
		f.write(str(newData)+'\n')
		f.flush()
		if(len(newData) > 316):
			pub = rospy.Publisher("Maestro/Control", PythonMessage)
			pub.publish("RSP", "position", "-1.0", "")
		
	except:
		pub = rospy.Publisher("Maestro/Control", PythonMessage)
		pub.publish("RSP", "position", "0.0", "")	
	
def closeFile():
	print "  "
	f.close()

print "Init"
f = open("audiolog.log", 'w')
subprocess.Popen(['xterm', '-e', 'roslaunch', 'audio_capture', 'capture.launch'])
time.sleep(.5)
rospy.init_node("Soundlistener")
rospy.Subscriber("/audio", AudioData, update)
rospy.on_shutdown(closeFile)
count = 0
print "Press Ctrl-C to stop listening"
while not rospy.is_shutdown():
	if(count == 0):
		sys.stdout.write('\r')
		sys.stdout.flush()
		sys.stdout.write("listening.     ")
		sys.stdout.flush()
		time.sleep(.25)
		count += 1
	if(count == 1):
		sys.stdout.write('\r')
		sys.stdout.flush()
		sys.stdout.write("listening..    ")
		sys.stdout.flush()
		time.sleep(.25)
		count += 1
	if(count == 2):
		sys.stdout.write('\r')
		sys.stdout.flush()
		sys.stdout.write("listening...   ")
		sys.stdout.flush()
		time.sleep(.25)
		count = 0

