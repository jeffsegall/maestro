#!/usr/bin/env python
from Tkinter import *
from MaestroController import MaestroController

class ControlWindow (Frame):
	
	def __init__(self, parent):
		Frame.__init__(self, parent)
		self.parent = parent
		self.controller = MaestroController()
		self.initUI()
		

	def sendCommand(self):
		try:
			joint=self.jointbox.get()
			command=self.propertybox.get()
			value=float(self.valueEntry.get())
 			self.controller.publishMessage(joint, command, value, "")
		except ValueError:
			pass

	def getInfo(self):
		try:
			joint=self.jointbox2.get()
			prop= self.propertyentry.get()
			value = self.controller.get(joint, prop)
			self.battery.set(""+ joint + " " + prop + ": " + str(value))
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
		self.battery.set("Information appears here")
		
		frame = Frame(self, relief=RAISED, borderwidth=1)
		frame.grid(row=0, columnspan=4,sticky=W+E)
		BatteryLabel = Label(frame, textvariable=self.battery)
		BatteryLabel.grid(row=0, column = 0)	
		self.jointbox = Spinbox(self, values=jointTuple, text="Joint")
		self.jointbox.grid(row=2, column=0)
		
		self.propertybox = Spinbox(self, values=("position", "velocity"), state="readonly", text="Property")
		self.propertybox.grid(row=2, column=1)

		#self.checkVar = IntVar()
		#checkButton = CheckButton(self, text="Enable", variable=self.checkVar, onvalue=1, offvalue=2)
		#checkButton.pack(fill=BOTH, expand=1, side=LEFT)

		self.dubVar = DoubleVar()

		self.valueEntry = Entry(self, text="value")
		self.valueEntry.grid(row=2, column=2)
		
		self.propertyentry = Entry(self, text="otherProperty")
		self.propertyentry.grid(row=5, column=1)

		commandButton = Button(self, text="Send", command=self.sendCommand)
		commandButton.grid(row=2, column=3)
		
		getButton = Button(self, text="Get", command=self.getInfo)
		getButton.grid(row=5, column=3)

		jointLabel = Label(self, text="Joint: ")
		jointLabel.grid(row=1, column=0)

		propertyLabel = Label(self, text="Property: ")
		propertyLabel.grid(row=1, column = 1)

		valueLabel = Label(self, text="Value: ")
		valueLabel.grid(row=1, column=2)
		
		dividerFrame = Frame(self, height=3, bg="black", relief=RAISED, borderwidth=1)
		dividerFrame.grid(row=3, columnspan=4, sticky=W+E)
			
		jointLabel2 = Label(self, text="Joint: ")
		jointLabel2.grid(row=4, column=0)

		propertyLabel2 = Label(self, text="Property: ")
		propertyLabel2.grid(row=4, column = 1)

		self.jointbox2 = Spinbox(self, values=jointTuple, text="GetJoint")
		self.jointbox2.grid(row=5, column=0)


		dividerFrame = Frame(self, height=3, bg="black", relief=RAISED, borderwidth=1)
		dividerFrame.grid(row=6, columnspan=4, sticky=W+E)
		
		initRobotButton = Button(self, text="initRobot", command=self.initRobot)
		initRobotButton.grid(row=7, column=0)


		homeAllButton = Button(self, text="HomeAll", command=self.homeAll)
		homeAllButton.grid(row=7, column=1)
			

		enableButton = Button(self, text="EnableAll", command=self.enableAll)
		enableButton.grid(row=7, column=2)


		initSensorsButton = Button(self, text="InitSensors", command=self.initSensors)
		initSensorsButton.grid(row=7, column=3)

	def initRobot(self):
		self.controller.initRobot()

	def homeAll(self):
		self.controller.homeAll()

	def enableAll(self):
		self.controller.enableAll()
	
	def initSensors(self):
		self.controller.initSensors()
if __name__ == "__main__":
	root = Tk()
	root.geometry("610x150+300+300")
	frame = ControlWindow(root)
	root.mainloop()
