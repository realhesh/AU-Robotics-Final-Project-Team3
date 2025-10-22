import pygame
import threading
import time
import json
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QBrush

class JoystickWidget(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self._x = 0.0
        self._y = 0.0
        self._rx = 0.0
        self._lx = 0.0

        
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        
        self.running = True
        threading.Thread(target=self._joystick_loop, daemon=True).start()

    def _joystick_loop(self):
        while self.running:
            pygame.event.pump()
            x = self.joystick.get_axis(0)
            y = self.joystick.get_axis(1)
            r_y = self.joystick.get_axis(2)
            r_x = self.joystick.get_axis(3)

            self._x = x
            self._y = y
            self._rx = r_x
            self._ry = r_y
            self.update()  # repaint
            time.sleep(0.05)

    def paintEvent(self, event):
        painter = QPainter(self)
        # for Smoothing
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # rectangle -> represent whole widget
        painter.setBrush(QBrush(QColor(0 , 0 , 0)))
        painter.drawRect(self.rect())

        # ellipse -> the cover of analog of joystick
        self._lcenterx = self.width() / 4
        self._lcentery = self.height() / 2

        painter.setBrush(QBrush(QColor(180 , 0 , 0)))
        self._ellipse_rad = min(self.width() , self.height()) / 5

        painter.drawEllipse(self._lcenterx - self._ellipse_rad , self._lcentery - self._ellipse_rad , self._ellipse_rad * 2 , self._ellipse_rad * 2) # type: ignore
        
        # ellipse -> analog of joystick
        self._analog_rad = 20
        self._lmovex = self._lcenterx + self._x * (self._ellipse_rad - self._analog_rad)
        self._lmovey = self._lcentery + self._y * (self._ellipse_rad - self._analog_rad)

        painter.setBrush(QBrush(QColor(255, 50, 50)))
        painter.drawEllipse(self._lmovex - self._analog_rad , self._lmovey - self._analog_rad , self._analog_rad * 2 , self._analog_rad * 2) # type: ignore


        self._rcenterx = 3 * self.width() / 4
        self._rcentery = self.height() / 2

        painter.setBrush(QBrush(QColor(180 , 0 , 0)))
        

        painter.drawEllipse(self._rcenterx - self._ellipse_rad , self._rcentery - self._ellipse_rad , self._ellipse_rad * 2 , self._ellipse_rad * 2) # type: ignore
        
        # ellipse -> analog of joystick
    
        self._rmovex = self._rcenterx + self._rx * (self._ellipse_rad - self._analog_rad)
        self._rmovey = self._rcentery + self._ry * (self._ellipse_rad - self._analog_rad)

        painter.setBrush(QBrush(QColor(255, 50, 50)))
        painter.drawEllipse(self._rmovex - self._analog_rad , self._rmovey - self._analog_rad , self._analog_rad * 2 , self._analog_rad * 2) # type: ignore    



    def stop(self):
        self.running = False
        pygame.quit()