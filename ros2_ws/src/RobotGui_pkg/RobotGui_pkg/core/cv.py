import cv2
from threading import Thread
from time import sleep
import os
from ament_index_python.packages import get_package_share_directory #i need this to go to where my pkg runs from install folder
class Camera:
    def __init__(self) -> None:
        self._PkgPath = get_package_share_directory('RobotGui_pkg')
        self._ImgPath = os.path.join(self._PkgPath,'pics','nocam.jpg')
        self._EmptyFrame = cv2.imread(self._ImgPath)
        #print(pic_path)
        self._CamIp = 'http://192.168.1.15:4747/video'
        self._cap = cv2.VideoCapture(self._CamIp)
        self._cap.open(self._CamIp)
        self._frame = None
        self._frame_thread = Thread(target=self._frame_loop, daemon=True)
        self._frame_thread.start()

    def _frame_loop(self):
        while True:
            success, image = self._cap.read()
            if success:
                self._frame = image
            else:
                self._cap.release()
                self._cap.open(self._CamIp)
            
            sleep(0.03)
            

    @property
    def frame(self):
        return self._frame if self._frame is not None else self._EmptyFrame