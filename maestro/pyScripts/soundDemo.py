#!/usr/bin/env python

import roslib; roslib.load_manifest('hubomsg')
import rospy
import alsaaudio
import sys
import time
import audioop
from hubomsg.msg import PythonMessage

if __name__ == '__main__':
	#Initialize and set the properties of PCM object
	card = 'default'
	audioInput = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, card)
	audioInput.setchannels(2)
	audioInput.setrate(44100)
	audioInput.setformat(alsaaudio.PCM_FORMAT_S16_LE)
	audioInput.setperiodsize(160)
	#Initialize ros node and get a publisher from it
	rospy.init_node("Noise_listener")
	pub = rospy.Publisher("Maestro/Control", PythonMessage)
	try:
		#Start an infite loop that gets and analyzes audio data
		while True:
			l, data = audioInput.read()
			if l:
				maxVal = audioop.max(data, 2)
				# check maxVal vs two thresholds 
				if maxVal > 500: 
					position =(-3.14 * maxVal/1000.0)
					if(position < -3.14):
						position = -3.14
					pub.publish("RSP", "position", str(position), "")
				elif maxVal < 70:
					position = 0
					pub.publish("RSP", "position", str(position), "")
				time.sleep(.001) #audio refresh rate
	except KeyboardInterrupt :
		sys.exit() #TODO make it actually exit
			
