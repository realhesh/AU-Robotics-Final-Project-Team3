from PySide6.QtWidgets import QWidget, QMainWindow , QVBoxLayout , QLabel , QHBoxLayout
from PySide6.QtCore import QTimer
from RobotGui_pkg.gui.CameraDisplay import CameraDisplay
from PySide6.QtCore import Qt , QRect
from PySide6.QtGui import QPalette , QColor , QPainter , QMovie
from RobotGui_pkg.gui.Arm_Dropoff_coordinates import ArmDropoffCordinatesWidget, Arm_Dropoff_Logic
from RobotGui_pkg.gui.JoyStickDisplay import JoystickWidget
from RobotGui_pkg.core.comm.GuiRosNode import GuiRosNode
from ament_index_python.packages import get_package_share_directory #i need this to go to where my pkg runs from install folder
import os

class Window(QMainWindow):
    def __init__(self ,ros_node:GuiRosNode | None = None):
        super().__init__()
        #
        self._PkgPath = get_package_share_directory('RobotGui_pkg')
        self._ImgPath = os.path.join(self._PkgPath,'pics','cow-cows.gif')
        #self.movie = QMovie(self._ImgPath)
        #self.movie.frameChanged.connect(self.update)  
        #self.movie.start()
        self.setCentralWidget(CentralWidget(ros_node=ros_node))
        self.setMinimumSize(1280, 720)
        self.show()
    # def paintEvent(self, event):
    #     painter = QPainter(self)
    #     current_frame = self.movie.currentPixmap()
    #     if not current_frame.isNull():
    #         painter.drawPixmap(self.rect(), current_frame)
    #     super().paintEvent(event)
  
    

        

class CentralWidget(QWidget):
    def __init__(self,parent:QWidget | None = None,ros_node:GuiRosNode | None = None):
        super().__init__(parent)
        self.ros_node = ros_node
        self._mainVBox = QVBoxLayout(self)
        self._ArmDropoffCoordinatesWidget = ArmDropoffCordinatesWidget(data = Arm_Dropoff_Logic(),ros_node=ros_node)
        self._CameraWidget = CameraDisplay(coords_widget=self._ArmDropoffCoordinatesWidget)
        self._CameraWidget.setMinimumSize(640, 480)
        self._mainVBox.addWidget(self._CameraWidget,alignment=Qt.AlignCenter)
        self._DataHBox = QHBoxLayout()
        self._mainVBox.addLayout(self._DataHBox)
        
        self._DataHBox.addWidget(self._ArmDropoffCoordinatesWidget)
        self._JoyStickWidget = JoystickWidget()
        self._DataHBox.addWidget(self._JoyStickWidget)
        #elf.ros_node.get_logger().info(f"{self._CameraWidget.size()}")
        self._CameraTimer = QTimer()
        self._CameraTimer.timeout.connect(self._CameraWidget.update_view)
        self._CameraTimer.setInterval(50)
        self._CameraTimer.start()
    # def update(self):
    #     self._itr+=1
    #     self._label.setText(f"Hello, Robot GUI! {self._itr}")
    #     self._label.setText(f"Hello, Robot GUI! {self._itr}")
