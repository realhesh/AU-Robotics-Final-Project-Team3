from PySide6.QtWidgets import QWidget, QLabel, QSizePolicy , QHBoxLayout
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt
from RobotGui_pkg.core.cv import Camera
#from Qt import AlignCenter
class CameraDisplay(QWidget):
    def __init__(self, parent: QWidget| None = None):
        super().__init__(parent)
        self._camera_device = Camera()
        self._CamHBox = QHBoxLayout(self)
    
        self._FrameView = QLabel()
        self._FrameView.setScaledContents(True)
        self._ProcessedView = QLabel()
        self._ProcessedView.setScaledContents(True)
        self._CamHBox.addWidget(self._FrameView)
        self._CamHBox.addWidget(self._ProcessedView)

        #self._frame_view.setStyleSheet("QLabel {background-color: red;}")
        self.update_view()

    def update_view(self):
        frame = self._camera_device.frame
        image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format.Format_BGR888)
        self._FrameView.setPixmap(QPixmap.fromImage(image))
        processed_frame = self._camera_device.ProcessedFrame
        processed_image = QImage(processed_frame.data, processed_frame.shape[1], processed_frame.shape[0], processed_frame.strides[0], QImage.Format.Format_BGR888)
        self._ProcessedView.setPixmap(QPixmap.fromImage(processed_image))