import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from my_robot_utils.custom_logger import NodeLogger

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller_node")
        self.logger_ = NodeLogger("motor_controller_node", logging_level='INFO')
        self.detected_obj_subscriber_ = self.create_subscription(
            Float64MultiArray,
            "object_position",
            self.calculate_cmd_vel_data,
            10
        )
        self.logger_.log_info("Subscription created to 'object_position' topic")
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.logger_.log_info("Publisher has been created on topic 'cmd_vel'.")
        
        self.logger_.log_info("Motor controller node has been started.")
        
        # Proportional control gain
        self.KP = 0.01  # Proportional gain, adjust as necessary

        # Linear velocity
        self.LINEAR_VELOCITY = 0.2  # Default linear velocity (in m/s)

        # Distance threshold
        self.MIN_DISTANCE_THRESHOLD = 0.25  # Minimum distance (in m) to stop the robot

        self.logger_.log_info(f"Constant parameters are:\n"
                              f"Kp = {self.KP}\n"
                              f"Linear Velocity = {self.LINEAR_VELOCITY}\n"
                              f"Min Distance Threshold = {self.MIN_DISTANCE_THRESHOLD}")
        self.get_logger().info("motor_controller_node started.")
        
        
    def calculate_cmd_vel_data(self, msg: Float64MultiArray):
        try:
            offset = msg.data[0]
            distance = msg.data[1]
            
            cmd = Twist()
            
            if distance <= self.MIN_DISTANCE_THRESHOLD:
                cmd.linear.x = 0.0
                if not (-15 <= offset <= 15):
                    cmd.angular.z = -self.KP * offset
                else:
                    cmd.angular.z = 0.0
            elif (-15 <= offset <= 15):
                cmd.linear.x = self.LINEAR_VELOCITY
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = self.LINEAR_VELOCITY
                cmd.angular.z = -self.KP * offset
                
            self.cmd_vel_publisher_.publish(cmd)
            self.logger_.log_debug(f"cmd.linear.x = {cmd.linear.x} and cmd.angular.z = {cmd.angular.z} is published on 'cmd_vel'.")
        except Exception as e:
            self.logger_.log_error(f"Failed to calculate and publish cmd_vel data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down 'motor_controller_node'...")
        node.logger_.log_info("Shutting down 'motor_controller_node'...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()