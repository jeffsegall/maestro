#!/usr/bin/env python
from Tkinter import *
from MaestroController import MaestroController

class ControlWindow (Frame):
	
	def __init__(self, parent):
		Frame.__init__(self, parent)
		self.parent = parent
		#self.controller = MaestroController()
		self.initUI()
		

	def sendCommand(self):
		try:
			joint=self.jointbox.get()
			command=self.propertybox.get()
			value=float(self.valueEntry.get())
 			self.controller.publishMessage(joint, command, value, "")
		except ValueError:
			pass
	def initUI(self):
		self.parent.title("Maestro")
		self.pack(fill=BOTH, expand=1)
		jointTuple = ("WST", "NKY", "NK1", "NK2",
                "LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP",
                "RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP",
                "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
                "RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
                "RF1", "RF2", "RF3", "RF4", "RF5",
                "LF1", "LF2", "LF3", "LF4", "LF5")

		self.battery = StringVar()
		self.battery = "Battery voltage"
		
		frame = Frame(self, relief=RAISED, borderwidth=1)
		frame.pack(fill=BOTH, expand=1, side=TOP)
		BatteryLabel = Label(frame, text="Battery voltage", textvariable=self.battery)
		BatteryLabel.pack(fill=BOTH, expand=1,side=LEFT)		
		
		self.jointbox = Spinbox(self, values=jointTuple, text="Joint")
		self.jointbox.pack(fill=X, expand=1, side=LEFT)
		
		self.propertybox = Spinbox(self, values=("position", "velocity"), state="readonly", text="Property")
		self.propertybox.pack(fill=X, expand=1, side=LEFT)

		#self.checkVar = IntVar()
		#checkButton = CheckButton(self, text="Enable", variable=self.checkVar, onvalue=1, offvalue=2)
		#checkButton.pack(fill=BOTH, expand=1, side=LEFT)

		self.dubVar = DoubleVar()

		self.valueEntry = Entry(self, text="value")
		self.valueEntry.pack(fill=X, expand=1, side=LEFT)

		commandButton = Button(self, text="Send", command=self.sendCommand)
		commandButton.pack(fill=X, expand=1, side=BOTTOM)
		
			

if __name__ == "__main__":
	root = Tk()
	root.geometry("700x100+300+300")
	frame = ControlWindow(root)
	root.mainloop()
