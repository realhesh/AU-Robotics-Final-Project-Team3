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
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"joystick detected: {self.joystick.get_name()}")

        self.gripper_angle=0
        
        thread=threading.Thread(target=self.listen_joystick)
        thread.daemon=True
        thread.start()

    def listen_joystick(self):
        try:
            while rclpy.ok():
                pygame.event.pump()
                #left joystick for motion
                left_y_axis = -self.joystick.get_axis(1) #up/down   
                left_x_axis = self.joystick.get_axis(0)  #left/right
                left_motor = left_y_axis + left_x_axis
                right_motor = left_y_axis - left_x_axis
                left_motor = max(min(left_motor, 1), -1)*255
                right_motor = max(min(right_motor, 1), -1)*255
                #gripper up and down
                vertical_gripper=-self.joystick.get_axis(3) #up/dowm
                vertical_gripper=max(min(vertical_gripper, 1), -1)*255
                #gripper control buttons for open/close
                square_button=self.joystick.get_button(0) #square for closing the gripper
                x_button=self.joystick.get_button(1) #x for opening the gripper
                circle_button=self.joystick.get_button(2) #circle
                L1_button=self.joystick.get_button(4) #L1 for reseting 
                R1_button=self.joystick.get_button(5) #R1 for saving location
                #hna feh two options 0 degrees means that gripper is opened and 180 degrees mean that gripper is close or
                #180 degrees means that gripper is opened and 0 degrees mean that gripper is closed hsb elservo
                if square_button:  # Close gripper
                    self.gripper_angle = min(self.gripper_angle + 5, 180) #lsa hzbt l part da 3shan msh 3rfa hwa mafroud elgripper max yfth ad eh 3shan yb2a 3la ad elbox belzbt
                if x_button:       # Open gripper
                    self.gripper_angle = max(self.gripper_angle - 5, 0)
        
                msg = String()
                msg.data = f"left:{left_motor}, right:{right_motor}, gripper_position:{vertical_gripper}, gripper_servo:{self.gripper_angle}, circle_button:{circle_button},reseting:{L1_button},saving_location:{R1_button}"
            
            
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing: {msg.data}")
            
                pygame.time.wait(100)
        except Exception as e:
            self.get_logger().error(f"Error in joystick thread: {e}")



def main(args=None):
    rclpy.init(args=args)
    node=ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
