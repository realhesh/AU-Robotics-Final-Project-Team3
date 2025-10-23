from PySide6.QtWidgets import QWidget, QLabel, QSizePolicy 
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt
from RobotGui_pkg.core.cv import Camera
from RobotGui_pkg.gui.Arm_Dropoff_coordinates import ArmDropoffCordinatesWidget
import RobotGui_pkg.core.QR_scanning.QR_scanning  as QR_scanning
#from Qt import AlignCenter
class CameraDisplay(QWidget):
    def __init__(self, parent: QWidget| None = None,coords_widget:ArmDropoffCordinatesWidget| None = None):
        super().__init__(parent)
        self.coords_widget = coords_widget
        self._camera_device = Camera()
        self._frame_view = QLabel(self)
        #self._frame_view.setMinimumSize(1280,720)
        self._frame_view.setScaledContents(True)
        #self._frame_view.setStyleSheet("QLabel {background-color: red;}")
        self.update_view()

    def update_view(self):
        frame = self._camera_device.frame
        image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format.Format_BGR888)
        if self._camera_device is not None :
            x,y,is_valid,is_blue = QR_scanning.QR_reader(frame)
        #print(f"i caught image that is {is_valid} and {is_blue} blue , x: {x} , y: {y}")
        if self._camera_device.frame is not None and is_valid == 'valid_coordinates' and is_blue:
            self.coords_widget.target_X = x
            self.coords_widget.target_Y = y
            self.coords_widget.update_display()
        self._frame_view.setPixmap(QPixmap.fromImage(image))