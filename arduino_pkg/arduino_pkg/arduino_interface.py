import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json
from datetime import datetime
from my_robot_utils.custom_logger import NodeLogger
import os

class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__("arduino_interface_node")
        self.logger_ = NodeLogger("arduino_interface_node", logging_level='INFO')
        self.cmd_vel_subscriber_ = self.create_subscription(Twist, "cmd_vel", self.send_commands_to_arduino, 10)
        self.logger_.log_info("Subscription has been created to topic 'cmd_vel'.")
        self.logger_.log_info("Establishing connection with Arduino...")
        
        self.ser = None
        self.check_ser_connection_timer_ = None
        self.declare_parameter('log_cmd_vel_values', False)
        self.log_cmd_vel_values = self.get_parameter('log_cmd_vel_values').value
        self.logger_.log_info(f"log_cmd_vel_values is set to: {self.log_cmd_vel_values}")
        
        self.cmd_vel_list = []
        if self.log_cmd_vel_values:
            self.log_dir = os.path.expanduser(f'~/robot_logs/arduino_interface_node_logs/')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file_name = f"{self.log_dir}cmd_vel_data_{timestamp}.json"
            self.logger_.log_info(f"Logging /cmd_vel data enabled, file: {self.log_file_name}")
        
        # Try to establish serial connection immediately
        self._check_serial_connection()
        # Set up a timer to retry if the initial connection fails
        if not self.ser or not self.ser.is_open:
            self.check_ser_connection_timer_ = self.create_timer(5, self._check_serial_connection)
            self.logger_.log_info("A timer is created to retry checking serial connection every 5 sec until it succeeds.")
    
    def _check_serial_connection(self):
        try:
            if self.ser is None or not self.ser.is_open:
                self.logger_.log_info("Checking serial communication...")
                self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                self.logger_.log_info("Serial Connection OK!!!")
                if self.check_ser_connection_timer_:
                    self.check_ser_connection_timer_.cancel()
                    self.check_ser_connection_timer_ = None
                    self.logger_.log_info("Timer is cancelled.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.get_logger().error("There is an error in Serial Communication. Please check the connection.")
    
    def save_to_file(self):
        with open(self.log_file_name, 'w') as file:
            json.dump(self.cmd_vel_list, file)
        self.logger_.log_info(f'Saved cmd_vel data to {self.log_file_name}')

    def send_commands_to_arduino(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.logger_.log_debug(f"linear velocity is {linear_x} and angular velocity is {angular_z}")
        
        if self.log_cmd_vel_values:
            self.cmd_vel_list.append({"linear_x": linear_x, "angular_z": angular_z})
        
        if self.ser and self.ser.is_open:
            # Send velocity commands to Arduino
            command = f"{linear_x},{angular_z}\n"
            self.ser.write(command.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down the node...")
        node.logger_.log_info("destroying the node...")
    finally:
        if node.log_cmd_vel_values:
            node.save_to_file()
        if node.ser:
            node.ser.close()
            node.logger_.log_info("Serial Connection is closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
