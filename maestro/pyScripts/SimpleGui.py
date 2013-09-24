#!/usr/bin/env python
from Tkinter import *
from MaestroController import MaestroController
class ControlWindow(Frame):
	def __init__(self, parent):
		Frame.__init__(self, parent)
		self.parent = parent
		self.controller = MaestroController()
		self.controller.executeCommonStartUp()
		self.initUI()
	def sendCommand(self):
		value = self.valueEntry.get()
		self.controller.publishMessage("RSP", "position", value, "")
	def initUI(self):
		self.parent.title("Maestro")
		self.pack(fill=BOTH, expand=1)
		self.valueEntry = Entry(self, text="value")
		self.valueEntry.grid(row=0, column =0)
		self.button = Button(self, text="Send", command = self.sendCommand)
		self.button.grid(row=0, column=1)
if __name__ == "__main__":
	root = Tk()
	root.geometry("150x150+100+100")
	frame = ControlWindow(root)
	root.mainloop()
