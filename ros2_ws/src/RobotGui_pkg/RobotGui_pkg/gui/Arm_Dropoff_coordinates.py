from email.mime import message
import math 
from PySide6.QtWidgets import QWidget, QLabel, QGridLayout , QVBoxLayout
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt
import paho.mqtt.client as mqtt 
from RobotGui_pkg.core.comm.GuiRosNode import GuiRosNode
import time 
class Arm_Dropoff_Logic:
    def __init__(self):
        self.target_X = 0.0
        self.target_Y = 0.0
        
    def Calcute_Error_In_Distance(self, X_current, Y_current):
        delta_X = self.target_X - X_current 
        delta_y = self.target_Y - Y_current 
        
        Error_In_Distance = math.sqrt(delta_X**2 + delta_y**2)
        return Error_In_Distance 
    
    def Calcute_Error_In_Angle(self, X_current, Y_current, Theta_Current):
        delta_X = self.target_X - X_current
        delta_y = self.target_Y - Y_current
        Theta_Required = math.atan2(delta_y, delta_X) 
        Theta_Current_Rad = math.radians(Theta_Current)
        delta_theta_rad = Theta_Required - Theta_Current_Rad 
        
        delta_theta_rad = math.fmod(delta_theta_rad + math.pi, 2 * math.pi) - math.pi
        Theta = math.degrees(delta_theta_rad)
        return Theta
    
    def Set_Target_Coordinates(self, target_X, target_Y):
        self.target_X = target_X 
        self.target_Y = target_Y

class ArmDropoffCordinatesWidget(QWidget):

    def __init__(self, data: Arm_Dropoff_Logic, ros_node : GuiRosNode | None = None):

        super().__init__()

        if not isinstance(data, Arm_Dropoff_Logic):
            raise TypeError("data must be an instance of Arm_Dropoff_Logic")
        self.ros_node = ros_node
        self.data = data
        layout = QGridLayout() 
        self.setLayout(layout)
        self.client = mqtt.Client()
        self.broker = "localhost"  
        self.port = 1883
        self.topic = "AUR/localization"
        self.Start_Subscriber()

        self.target_X = 0.0
        self.target_Y = 0.0

        self.x_value = 0.0
        self.y_value = 0.0
        self.angle_value = 0.0
        
        self.x_value_label = QLabel("X: 0.00 cm")
        self.x_value_label.setFont(QFont("Arial", 15))
        self.x_value_label.setStyleSheet("color : green")
           

        self.y_value_label = QLabel("Y: 0.00 cm")
        self.y_value_label.setFont(QFont("Arial", 15))
        self.y_value_label.setStyleSheet("color : green")
        

        self.angle_value_label = QLabel("Angle: 0.00 degree")
        self.angle_value_label.setFont(QFont("Arial", 15))
        self.angle_value_label.setStyleSheet("color : green")
        
        
        self.distance_error_label = QLabel("Distance Error: 0.00 cm")
        self.distance_error_label.setFont(QFont("Arial", 22 , QFont.Bold))
        self.distance_error_label.setStyleSheet("color: Red;")
        
        
        self.angle_error_label = QLabel("Angle Error: 0.00 degree")
        self.angle_error_label.setFont(QFont("Arial", 22 , QFont.Bold))
        self.angle_error_label.setStyleSheet("color: Red;")
        

        self.target_x_label = QLabel("Target X: 0.00 cm")
        self.target_x_label.setFont(QFont("Arial", 15 ))
        self.target_x_label.setStyleSheet("color: Blue;")
        
        
        self.target_y_label = QLabel("Target Y: 0.00 cm")
        self.target_y_label.setFont(QFont("Arial", 15 ))
        self.target_y_label.setStyleSheet("color: Blue;")
        

        self.Arm_Dropoff_label = QLabel("Arm Dropoff Coordinate : (0.00 , 0.00 )")
        self.Arm_Dropoff_label.setFont(QFont("Arial", 15 ))
        self.Arm_Dropoff_label.setStyleSheet("color: green;")
        layout.addWidget(self.x_value_label, 0, 0)
        layout.addWidget(self.y_value_label, 1, 0)
        layout.addWidget(self.angle_value_label, 2, 0)
        layout.addWidget(self.distance_error_label, 3, 0)
        layout.addWidget(self.angle_error_label, 4, 0)
        layout.addWidget(self.target_x_label, 5, 0)
        layout.addWidget(self.target_y_label, 6, 0)
        #layout.addWidget(self.Arm_Dropoff_label, 7, 0)
        
       
    def update_display(self):
        self.data.Set_Target_Coordinates(self.target_X, self.target_Y)
        self.x_value_label.setText(f"X : {self.x_value:.2f} cm")
        self.y_value_label.setText(f"Y : {self.y_value:.2f} cm")
        self.angle_value_label.setText(f"Angle : {self.angle_value:.2f} cm")

        self.target_x_label.setText(f"Target X : {self.target_X:.2f} cm")
        self.target_y_label.setText(f"Target Y : {self.target_Y:.2f} cm")

        self.distance_error_label.setText(f"Distance Error : {self.data.Calcute_Error_In_Distance(self.x_value,self.y_value):.2f} cm")
        self.angle_error_label.setText(f"Angle Error : {self.data.Calcute_Error_In_Angle(self.x_value,self.y_value,self.angle_value):.2f} degree")

    def Start_Subscriber(self):

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
    
        try :
            self.client.connect(self.broker,self.port,60)
        except :
            print("An Error Has Occured")
            return
        self.client.loop_start()


    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        self.ros_node.get_logger().info(f"Received from MQTT: {msg.topic} -> {message}")
        
        # Example: assume message contains "x,y"
        try:
            x_str, y_str, theta_str = message.split(",")
            self.x_value = float(x_str)
            self.y_value = float(y_str)
            self.angle_value = float(theta_str)
            self.data.Set_Target_Coordinates(self.target_X, self.target_Y)
            self.update_display()
        except Exception as e:
            self.ros_node.get_logger().warn(f"Failed to parse coordinates: {message} ({e})")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.ros_node.get_logger().info("MQTT connected successfully")
            client.subscribe(self.topic)
            self.ros_node.get_logger().info(f"Subscribed to topic: {self.topic}")
        else:
            self.ros_node.get_logger().error(f"MQTT connection failed with code: {rc}")



