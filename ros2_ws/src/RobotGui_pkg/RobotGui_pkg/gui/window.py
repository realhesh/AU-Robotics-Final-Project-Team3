from PySide6.QtWidgets import QWidget, QMainWindow , QVBoxLayout , QLabel 
from PySide6.QtCore import QTimer
#from RobotGui.gui.camera_display import CameraDisplay
#from RobotGui.gui.coordinates_display import CoordinatesDisplay
#from RobotGui.core.comm.client import setup
#from RobotGui.core.cv import Camera
from RobotGui_pkg.core.comm.GuiRosNode import GuiRosNode
class Window(QMainWindow):
    def __init__(self ,ros_node:GuiRosNode | None = None):
        super().__init__()
        self.setCentralWidget(CentralWidget(ros_node=ros_node))
        self.setMinimumSize(1280, 720)
        self.show()

        

class CentralWidget(QWidget):
    def __init__(self,parent:QWidget | None = None,ros_node:GuiRosNode | None = None):
        super().__init__(parent)
        self._label = QLabel(self)
        self._itr = 0
        self._label.setText(f"Hello, Robot GUI! {self._itr}")
        self.ros_node = ros_node
        self.ros_node.timer = self.ros_node.create_timer(1.0, self.update)
        # self._vbox.addWidget(self._coords_widget)
        # self._camera_timer = QTimer()
        # self._camera_timer.timeout.connect(self._camera_widget.update_view)
        # self._camera_timer.setInterval(50)
        # self._camera_timer.start()
    def update(self):
        self._itr+=1
        self._label.setText(f"Hello, Robot GUI! {self._itr}")
        self._label.setText(f"Hello, Robot GUI! {self._itr}")
