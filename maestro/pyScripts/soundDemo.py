#!/usr/bin/env python

import roslib; roslib.load_manifest('hubomsg')
import rospy
import alsaaudio
import sys
import time
import audioop
from hubomsg.msg import PythonMessage

if __name__ == '__main__':
	card = 'default'
	audioInput = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, card)
	audioInput.setchannels(2)
	audioInput.setrate(44100)
	audioInput.setformat(alsaaudio.PCM_FORMAT_S16_LE)
	audioInput.setperiodsize(160)
	rospy.init_node("Noise_listener")
	pub = rospy.Publisher("Maestro/Control", PythonMessage)
	try:
		while True:
			l, data = audioInput.read()
			if l:
				maxVal = audioop.max(data, 2)
				if maxVal > 1000:
					pub.publish("RSP", "position", "-1.0", "")
				elif maxVal < 100:
					pub.publish("RSP", "position", "0.0", "")
				time.sleep(.001)
	except KeyboardInterrupt :
		sys.exit()
	except : 
		pass
		
