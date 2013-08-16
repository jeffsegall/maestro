from MaestroController import MaestroController
import rlcompleter
import readline
readline.parse_and_bind("tab: complete")
controller = MaestroController()
print("Welcome to the MAESTRO python interpreter!\nTo command the robot call the methods associated with MaestroController on controller.\nEg: controller.initRobot()")
