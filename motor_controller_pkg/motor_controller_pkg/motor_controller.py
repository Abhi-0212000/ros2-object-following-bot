import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller_node")
        self.detected_obj_subscriber_ = self.create_subscription(
            Float64MultiArray,
            "object_position",
            self.calculate_cmd_vel_data,
            10
        )
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("Motor controller node has been started.")
        
        # Proportional control gain
        self.Kp = 0.01  # Proportional gain, adjust as necessary

        # Linear velocity
        self.linear_velocity = 0.2  # Default linear velocity (in mts/sec)

        # Distance threshold
        self.min_distance_threshold = 0.25  # Minimum distance (in mts) to stop the robot
        
        
    def calculate_cmd_vel_data(self, msg: Float64MultiArray):
        
        offset = msg.data[0]
        distance = msg.data[1]
        
        cmd = Twist()
        
        if distance <= self.min_distance_threshold:
            cmd.linear.x = 0.0
            if not (-15 <= offset <= 15):
                cmd.angular.z = -self.Kp * offset
            else:
                cmd.angular.z = 0.0
                
        elif (-15 <= offset <= 15):
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = -self.Kp * offset
            
        self.cmd_vel_publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()