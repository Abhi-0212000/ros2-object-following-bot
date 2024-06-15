import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__("arduino_interface_node")
        self.cmd_vel_subscriber_ = self.create_subscription(Twist, "cmd_vel", self.send_commands_to_arduino, 10)
        
        self.get_logger().info("Arduino node has been started \nEstablishing connection with Arduino...")
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Serial Connection OK!!!")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

        
    def send_commands_to_arduino(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f"linear velocity is {linear_x} and angular velocity is {angular_z}")
        
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
