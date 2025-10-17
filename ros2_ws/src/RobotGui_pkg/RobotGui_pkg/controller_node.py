import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import threading

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.publisher_=self.create_publisher(String,'controls',10)

        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count()==0:
            self.get_logger().error("No joystick detected!")
            return 
        self.joystick=pygame.joystick.Joystick()
        self.joystick.init()
        self.get_logger().info(f"joystick detected: {self.joystick.get_name()}")
        
        thread=threading.Thread(target=self.listen_joystick)
        thread.daemon=True
        thread.start()

    def listen_joystick(self):
        while rclpy.ok():
            pygame.event.pump()
            y_axis= -self.joystick.get_axis(1) #up/down   
            x_axis = self.joystick.get_axis(0)  #left/right
            
            
            left_motor = y_axis + x_axis
            right_motor =y_axis - x_axis
            
            
            left_motor = max(min(left_motor, 1), -1)
            right_motor = max(min(right_motor, 1), -1)
            
            msg = String()
            msg.data = f"left:{left_motor}, right:{right_motor}"
            
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing: {msg.data}")
            
            pygame.time.wait(50)  



def main(args=None):
    rclpy.init(args=args)
    node=ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
