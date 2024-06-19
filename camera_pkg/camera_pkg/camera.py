import rclpy
from rclpy.node import Node
import requests
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from my_robot_utils.logger import NodeLogger

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.logger_ = NodeLogger("camera_node")

        # Declare parameter to choose camera source
        self.declare_parameter('camera_source', 'android')
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        self.logger_.log_info(f"camera_source is {self.camera_source}")
        
        # Pass the IP address to access android cam feed
        self.declare_parameter('ip', '192.168.0.104')   # bash: ros2 run camera_pkg camera_node --ros-args -p "ip":="192.168.0.104"
        if self.get_parameter('ip').value is None:
            self.logger_.log_error("IP parameter not provided. Exiting...")
            return
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.logger_.log_info(f"ip = {self.ip}")
        
        # Declare parameter to control window display
        self.declare_parameter('show_window', False)
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.logger_.log_info(f"show_window is set to {self.show_window}")
        
        self.bridge = CvBridge()
        self.image_publisher_ = self.create_publisher(Image, "camera_image", 10)
        self.logger_.log_info(f"image_publisher_ is created on topic 'camera_image'.")

        # Initialize video capture based on camera source
        self.running = True
        self.retry_timer = None
        if self.camera_source == 'android':
            self.android_url = "http://" + self.ip + ":8080/shot.jpg"
            self.logger_.log_info(f"Android camera URL: {self.android_url}")
            self.init_android_cam()
        else:
            self.logger_.log_warning(f"Unknown camera source: {self.camera_source}")
            
        self.busy_reading_frames = False

        self.get_logger().info("Camera node has been started.")
        self.logger_.log_info("camera_node started")

    def init_android_cam(self):
        """Initialize the Android camera video capture using an IP stream."""
        # calling func at 25 times/sec ~= 25fpsi.e processing 25 image /sec 
        self.timer = self.create_timer(0.04, self.read_frames_from_android_cam)
        self.logger_.log_info("Initializing android camera...")

    def read_frames_from_android_cam(self):
        """Read and display frames from the Android camera using an IP stream."""
        if not self.running:
            return
        
        if self.busy_reading_frames:
            return  # Skip if already processing a frame read
        
        self.busy_reading_frames = True  # Mark that we're starting to process frames

        url = self.android_url
        new_width, new_height = 800, 400

        try:
            img_resp = requests.get(url)
            img_resp.raise_for_status()  # Raise HTTPError for bad responses
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            img = cv2.imdecode(img_arr, -1)

            # Resize the image
            resized_image = cv2.resize(img, (new_width, new_height))
            
            # Convert OpenCV image to ROS image message
            image_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")

            # Publish the image message
            self.image_publisher_.publish(image_msg)
            self.logger_.log_info("Published a frame from the Android camera.")

            if self.show_window:
                cv2.imshow("Android_cam", resized_image)
                if cv2.waitKey(1) == 27:  # Esc key has ASCII value of 27
                    self.running = False
                    self.logger_.log_info("Shutting down the Android_cam window.")
                    self.destroy_node()
        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error accessing Android camera: {e}")
            self.logger_.log_error(f"Error accessing Android camera: {e}")
            self.get_logger().error("Please ensure the IP webcam is started on your Android device.")
            self.logger_.log_warning("Please ensure the IP webcam is started on your Android device.")
            self.timer.cancel()
            self.timer = None  # Ensure the previous timer is null before setting a retry timer
            if self.retry_timer is None:
                self.retry_timer = self.create_timer(5.0, self.retry_init_android_cam)  # Retry after 5 seconds
        
        finally:
            self.busy_reading_frames = False  # Reset to allow next frame read

    def retry_init_android_cam(self):
        """Retry initializing the Android camera."""
        self.retry_timer.cancel()  # Cancel the retry timer to avoid multiple retry attempts
        self.retry_timer = None
        self.logger_.log_info("Retrying to initialize the Android camera...")
        self.init_android_cam()  # Try to reinitialize the Android camera

    def destroy_node(self):
        """Override the destroy_node method to release resources and close windows."""
        if self.timer is not None:
            self.timer.cancel()
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        cv2.destroyAllWindows()
        super().destroy_node()
        self.logger_.log_info("cv2 windows and camera_node are destroyed")

def main(args=None):
    """Main function to initialize the ROS 2 node and start spinning."""
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
