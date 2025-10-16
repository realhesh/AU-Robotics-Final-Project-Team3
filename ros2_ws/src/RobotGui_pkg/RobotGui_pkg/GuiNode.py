import sys
import threading
import rclpy 
import numpy
from PySide6.QtWidgets import QApplication
from RobotGui_pkg.gui.window import Window
from RobotGui_pkg.core.comm.GuiRosNode import GuiRosNode
if __package__ is None and not getattr(sys, 'frozen', False):
    import os.path
    path = os.path.realpath(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(os.path.dirname(path)))

def main():
    rclpy.init() 
    app = QApplication()
    gui_node = GuiRosNode() 
    window = Window(ros_node=gui_node)
    thread = threading.Thread(target=rclpy.spin,args=(gui_node,),daemon=True)
    thread.start()
    try:
        sys.exit(app.exec())
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()
        