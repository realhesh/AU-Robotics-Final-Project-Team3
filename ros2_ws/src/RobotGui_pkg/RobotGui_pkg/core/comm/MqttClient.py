# # RobotGui_pkg/core/comm/mqtt_client.py

# import paho.mqtt.client as mqtt
# import threading
# import time
# from RobotGui_pkg.gui import ArmDropoffCordinatesWidget

# class GuiMqttClient:
#     def __init__(self, ros_node, broker="10.96.185.112", port=1883 , coords: ArmDropoffCordinatesWidget | None = None):
#         self.ros_node = ros_node
#         self.broker = broker
#         self.port = port
#                 # You can customize these topics as you wish
#         self.pub_topic = "AUR/controls"
#         self.sub_topic = "AUR/localization"

#         # Create MQTT client
#         self.client = mqtt.Client()

#         # Assign callbacks
#         self.client.on_connect = self.on_connect
#         self.client.on_message = self.on_message

#         # Example internal data variables
#         self.Current_Coordinate_X = 0.0
#         self.Current_Coordinate_Y = 0.0
#         self.Current_Theta = 0.0
#         self.coords = coords

#     # ---------------- MQTT LIFECYCLE ----------------
#     def start(self):
#         """Start MQTT connection and loop in background."""
#         try:
#             self.client.connect(self.broker, self.port, 60)
#             self.client.loop_start()
#             self.ros_node.get_logger().info(f"MQTT client started on {self.broker}:{self.port}")
#         except Exception as e:
#             self.ros_node.get_logger().error(f"MQTT connection error: {e}")

#     def stop(self):
#         """Stop MQTT cleanly."""
#         self.client.loop_stop()
#         self.client.disconnect()
#         self.ros_node.get_logger().info("MQTT client stopped")

#     # ---------------- MQTT CALLBACKS ----------------
#     def on_connect(self, client, userdata, flags, rc):
#         if rc == 0:
#             self.ros_node.get_logger().info("MQTT connected successfully")
#             client.subscribe(self.sub_topic)
#             self.ros_node.get_logger().info(f"Subscribed to topic: {self.sub_topic}")
#         else:
#             self.ros_node.get_logger().error(f"MQTT connection failed with code: {rc}")

#     def on_message(self, client, userdata, msg):
#         message = msg.payload.decode()
#         self.ros_node.get_logger().info(f"Received from MQTT: {msg.topic} -> {message}")
        
#         # Example: assume message contains "x,y"
#         try:
#             x_str, y_str, theta_str = message.split(",")
#             self.Current_Coordinate_X = float(x_str)
#             self.Current_Coordinate_Y = float(y_str)
#             self.Current_Theta = float(theta_str)
#             self.coords.x_value =  self.Current_Coordinate_X
#             self.coords.y_value =  self.Current_Coordinate_Y
#             self.coords.angle_value = self.Current_Theta
#             self.coords.update_display()
#         except Exception as e:
#             self.ros_node.get_logger().warn(f"Failed to parse coordinates: {message} ({e})")

#         # Optionally store in ROS node
#         self.ros_node.Current_Coordinate_X = self.Current_Coordinate_X
#         self.ros_node.Current_Coordinate_Y = self.Current_Coordinate_Y

#     # ---------------- MQTT PUBLISHING ----------------
#     def publish_controls(self, control_msg: str):
#         """Publish control data to AUR/controls topic."""
#         try:
#             self.client.publish(self.pub_topic, control_msg)
#             self.ros_node.get_logger().info(f"Published to {self.pub_topic}: {control_msg}")
#         except Exception as e:
#             self.ros_node.get_logger().error(f"MQTT publish failed: {e}")