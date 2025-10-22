import math 
from PySide6.QtWidgets import QWidget, QLabel, QGridLayout , QVBoxLayout
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt


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

    def __init__(self, data: Arm_Dropoff_Logic):

        super().__init__()

        if not isinstance(data, Arm_Dropoff_Logic):
            raise TypeError("data must be an instance of Arm_Dropoff_Logic")
            
        self.data = data
        layout = QGridLayout() 
        self.setLayout(layout)
        
        
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
        layout.addWidget(self.Arm_Dropoff_label, 7, 0)
        
       
    def update_target_display(self):
          
        self.target_x_label.setText(f"Target X : {self.data.target_X:.2f} cm")
        self.target_y_label.setText(f"Target Y : {self.data.target_Y:.2f} cm")    

    
    def bluetooth_data(self, client, userinfo, message ):
        
        payload_str: str = message.payload.decode()
       
        coords = payload_str.split(',')
        
 
        

        try :    
            X_current = float(coords[0])
            Y_current = float(coords[1])
            Angle_current = float(coords[2])

            
            distance_error = self.data.Calcute_Error_In_Distance(X_current, Y_current)
            angle_error = self.data.Calcute_Error_In_Angle(X_current, Y_current, Angle_current)

            

            
            self.x_value_label.setText(f"X : {X_current : .2f} cm")
            self.y_value_label.setText(f"Y : {Y_current : .2f} cm")
            self.angle_value_label.setText(f"Angle : {Angle_current : .2f} degree")

            self.distance_error_label.setText(f"Distance Error: {distance_error : .2f} cm")
            self.angle_error_label.setText(f"Angle Error: {angle_error: .2f} degree")
            
            self.Arm_Dropoff_label.setText(f"Arm Dropoff Coordinates : ({self.data.target_X:.2f}, {self.data.target_Y:.2f})") 
        
        except ValueError as e:
            print(f"Error Processing Coordinates: {e}")
        



        

        


    
  


