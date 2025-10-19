import cv2
from threading import Thread
from time import sleep
import os
from ament_index_python.packages import get_package_share_directory #i need this to go to where my pkg runs from install folder
import numpy as np
class Camera:
    def __init__(self) -> None:
        self._PkgPath = get_package_share_directory('RobotGui_pkg')
        self._ImgPath = os.path.join(self._PkgPath,'pics','nocam.jpg')
        self._EmptyFrame = cv2.imread(self._ImgPath)
        #print(pic_path)
        self._CamIp = 'http://192.168.1.11:4747/video'
        self._cap = cv2.VideoCapture(self._CamIp)
        self._cap.open(self._CamIp)
        self._frame = None
        self._ProcessedFrame = None
        self._frame_thread = Thread(target=self._frame_loop, daemon=True)
        self._frame_thread.start()

    def _frame_loop(self):
        while True:
            success, image = self._cap.read()
            if success:
                self._frame = image
                self._ProcessedFrame = self._ProcessFrame(image)
                
            else:
                self._cap.release()
                self._cap.open(self._CamIp)
            
            sleep(0.03)
    def _ProcessFrame(self,image):
        img = image.copy()
        
        #blurs to blend colors together 
        ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        Y, Cr, Cb = cv2.split(ycrcb)
        Cr_blur = cv2.GaussianBlur(Cr, (5, 5), 0)
        Cb_blur = cv2.GaussianBlur(Cb, (5, 5), 0)
        ycrcb_blur = cv2.merge([Y, Cr_blur, Cb_blur])
        img = cv2.cvtColor(ycrcb_blur, cv2.COLOR_YCrCb2BGR)
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Define the red color range (tune if needed) #Update to whatever later 
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Combine both
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((11, 11), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours (edges of red areas)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around red objects
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # ignore small noise
                x, y, w, h = cv2.boundingRect(cnt)
                # Draw corner points
                cv2.circle(img, (x, y), 5, (0, 255, 0), -1)             # top-left
                cv2.circle(img, (x + w, y), 5, (0, 255, 0), -1)         # top-right
                cv2.circle(img, (x, y + h), 5, (0, 255, 0), -1)         # bottom-left
                cv2.circle(img, (x + w, y + h), 5, (0, 255, 0), -1)     # bottom-right
                #cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return img
        #return cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)


    @property
    def frame(self):
        return self._frame if self._frame is not None else self._EmptyFrame
    @property
    def ProcessedFrame(self):
        return self._ProcessedFrame if self._ProcessedFrame is not None else self._EmptyFrame