import rclpy
from rclpy.node import Node
import requests
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.get_logger().info("camera_node started!!!")

        # Declare parameter to choose camera source
        self.declare_parameter('camera_source', 'android')
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        
        # Pass the IP address to access android cam feed
        self.declare_parameter('ip', '192.168.0.104')   # bash: ros2 run camera_pkg camera_node --ros-args -p "ip":="192.168.0.104"
        if self.get_parameter('ip').value is None:
            self.get_logger().error("IP parameter not provided. Exiting...")
            return
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        
        self.bridge = CvBridge()
        self.image_publisher_ = self.create_publisher(Image, "camera_image", 10)

        # Initialize video capture based on camera source
        self.running = True
        self.retry_timer = None
        if self.camera_source == 'android':
            self.android_url = "http://" + self.ip + ":8080/shot.jpg"
            #self.get_logger().info(self.android_url)
            self.init_android_cam()

    def init_android_cam(self):
        """Initialize the Android camera video capture using an IP stream."""
        # calling func at 25 times/sec ~= 25fpsi.e processing 25 image /sec 
        self.timer = self.create_timer(0.04, self.read_frames_from_android_cam)

    def read_frames_from_android_cam(self):
        """Read and display frames from the Android camera using an IP stream."""
        if not self.running:
            return

        url = self.android_url
        new_width, new_height = 800, 400

        try:
            img_resp = requests.get(url)
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            img = cv2.imdecode(img_arr, -1)

            # Resize the image
            resized_image = cv2.resize(img, (new_width, new_height))
            
            # Convert OpenCV image to ROS image message
            image_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")

            # Publish the image message
            self.image_publisher_.publish(image_msg)

            cv2.imshow("Android_cam", resized_image)
            if cv2.waitKey(1) == 27:  # Esc key has ASCII value of 27
                self.running = False
                self.get_logger().info("Shutting down the Android_cam window...")
                self.get_logger().info("Press 'cntrl + c' to kill the node.")
                self.destroy_node()
            
        except requests.RequestException as e:
            self.get_logger().error(f"Error accessing Android camera: {e}")
            self.get_logger().error("Please ensure the IP webcam is started on your Android device.")
            self.timer.cancel()
            self.timer = None  # Ensure the previous timer is null before setting a retry timer
            if self.retry_timer is None:
                self.retry_timer = self.create_timer(5.0, self.retry_init_android_cam)  # Retry after 5 seconds

    def retry_init_android_cam(self):
        """Retry initializing the Android camera."""
        self.retry_timer.cancel()  # Cancel the retry timer to avoid multiple retry attempts
        self.retry_timer = None
        self.init_android_cam()  # Try to reinitialize the Android camera

    def destroy_node(self):
        """Override the destroy_node method to release resources and close windows."""
        if self.timer is not None:
            self.timer.cancel()
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """Main function to initialize the ROS 2 node and start spinning."""
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
