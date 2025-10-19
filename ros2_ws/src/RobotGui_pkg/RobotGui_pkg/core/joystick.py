import pygame
import threading
import time
import json
import paho.mqtt.client as mqtt
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QBrush



class JoystickWidget(QWidget):
    def __init__(self, broker="test.mosquitto.org", port=1883, topic="robot/joystick", parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self._x = 0.0
        self._y = 0.0

        
        self.client = mqtt.Client()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()

    
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

            self._x = x
            self._y = y

            self.update()  # repaint
            payload = json.dumps({"x": x, "y": y})
            self.client.publish(self.topic, payload)

            time.sleep(0.05)

    def paintEvent(self, event):
        painter = QPainter(self)
        # for Smoothing
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # rectangle -> represent whole widget
        painter.setBrush(QBrush(QColor(0 , 0 , 0)))
        painter.drawRect(self.rect())

        # ellipse -> the cover of analog of joystick
        self._centerx = self.width() / 2
        self._centery = self.height() / 2

        painter.setBrush(QBrush(QColor(180 , 0 , 0)))
        self._ellipse_rad = min(self.width() , self.height()) / 3

        painter.drawEllipse(self._centerx - self._ellipse_rad , self._centery - self._ellipse_rad , self._ellipse_rad * 2 , self._ellipse_rad * 2) # type: ignore
        
        # ellipse -> analog of joystick
        self._analog_rad = 30
        self._movex = self._centerx + self._x * (self._ellipse_rad - self._analog_rad)
        self._movey = self._centery + self._y * (self._ellipse_rad - self._analog_rad)

        painter.setBrush(QBrush(QColor(255, 50, 50)))
        painter.drawEllipse(self._movex - self._analog_rad , self._movey - self._analog_rad , self._analog_rad * 2 , self._analog_rad * 2) # type: ignore


    def stop(self):
        self.running = False
        pygame.quit()
        self.client.loop_stop()