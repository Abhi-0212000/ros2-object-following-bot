import rclpy
from rclpy.node import Node
import requests
import cv2
import numpy as np

class MultipleCamerasNode(Node):
    def __init__(self):
        super().__init__("multiple_cameras_node")
        self.get_logger().info("camera_node started!!!")

        # Declare parameter to choose camera source
        self.declare_parameter('camera_source', 'android')
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        
        # Pass the IP address to access android cam feed
        self.declare_parameter('ip', 'http://192.168.0.104:8080/')
        self.ip = self.get_parameter('ip').get_parameter_value().string_value

        # Initialize video capture based on camera source
        self.cap = None
        self.running = True
        self.retry_timer = None
        if self.camera_source == 'webcam':
            self.init_webcam()
        elif self.camera_source == 'android':
            self.android_url = self.ip + "shot.jpg"
            self.get_logger().info(self.android_url)
            self.init_android_cam()
        else:
            self.get_logger().error("Unknown camera source specified!")

    def init_webcam(self):
        """Initialize the webcam video capture."""
        self.cap = cv2.VideoCapture(0)  # self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Error opening video capture device!")
            return
        self.timer = self.create_timer(0.1, self.read_frames_from_webcam)

    def init_android_cam(self):
        """Initialize the Android camera video capture using an IP stream."""
        self.timer = self.create_timer(0.1, self.read_frames_from_android_cam)

    def read_frames_from_webcam(self):
        """Read and display frames from the webcam."""
        if not self.running:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error capturing frame!")
            return
        cv2.imshow('Webcam Feed', frame)
        if cv2.waitKey(1) == 27:  # Esc key has ASCII value of 27
            self.running = False
            self.get_logger().info("Shutting down...")
            self.destroy_node()

    def read_frames_from_android_cam(self):
        """Read and display frames from the Android camera using an IP stream."""
        if not self.running:
            return

        url = self.android_url
        new_width, new_height = 800, 600

        try:
            img_resp = requests.get(url)
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            img = cv2.imdecode(img_arr, -1)

            # Resize the image
            resized_image = cv2.resize(img, (new_width, new_height))

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

    def switch_camera_source(self, source):
        """Switch the camera source between webcam and Android camera."""
        if source not in ['webcam', 'android']:
            self.get_logger().error("Invalid camera source specified!")
            return
        self.camera_source = source
        self.get_logger().info(f"Switching to {source} camera")
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        cv2.destroyAllWindows()
        if source == 'webcam':
            self.init_webcam()
        elif source == 'android':
            self.init_android_cam()

    def destroy_node(self):
        """Override the destroy_node method to release resources and close windows."""
        if self.cap is not None:
            self.cap.release()
        if self.timer is not None:
            self.timer.cancel()
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """Main function to initialize the ROS 2 node and start spinning."""
    rclpy.init(args=args)
    node = MultipleCamerasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
