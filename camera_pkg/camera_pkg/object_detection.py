import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
import os

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.image_subscriber_ = self.create_subscription(Image, "camera_image", self.detect_object_from_image, 10)
        self.bridge = CvBridge()
        self.object_publisher_ = self.create_publisher(Float64, "object_position", 10)
        self.get_logger().info("Object detection node started.")
        
        # Load the Haar Cascade for face detection from the same directory as the script
        cascade_path = os.path.join(os.path.dirname(__file__), 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            self.get_logger().error("Failed to load Haar Cascade for face detection.")
        
        self.img_width, self.img_height = 800, 400
        # Calculate center coordinates
        self.img_center_x = int(self.img_width / 2)
        self.img_center_y = int(self.img_height / 2)
        
        # User preferences (can be set from UI or config)
        self.detected_face_visualization = True
        self.additional_visualizations = True

    def detect_object_from_image(self, img_msg: Image):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect faces in the image
            faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            
            # Check if any faces are detected
            if len(faces) > 0:
                # Get the first face (assuming single face detection)
                x, y, w, h = faces[0]

                # Calculate offset between detected face and image center
                offset = self.calculate_offset(x, w, self.img_center_x)
                
                # Publish the offset
                self.object_publisher_.publish(Float64(data=offset))
                
                # Draw visualizations and calculate offset (use combined draw function)
                cv_image = self.draw_visualizations(cv_image, x, y, w, h, offset)
            
            else:
                offset = 0
                # Publish the offset
                self.object_publisher_.publish(Float64(data=offset))
            
            # Display the image using OpenCV
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def calculate_offset(self, x, w, image_center_x):
        """
        This function calculates the horizontal offset between the bbox center and the vertical center line of the image.

        Args:
            x: Top-left x-coordinate of the bbox.
            w: Width of the bbox.
            image_center_x: X-coordinate of the image center.

        Returns:
            The horizontal offset (positive or negative) in pixels.
        """
        bbox_center_x = int(x + w / 2)
        offset = bbox_center_x - image_center_x
        return offset

    def draw_visualizations(self, image, x, y, w, h, offset):
        """
        This function draws additional visualizations based on user preference.

        Args:
            image: The OpenCV image object.
            x: Top-left x-coordinate of the bbox.
            y: Top-left y-coordinate of the bbox.
            w: Width of the bbox.
            h: Height of the bbox.
            offset: Horizontal offset between bbox center and image center.

        Returns:
            The image with additional visualizations (optional).
        """
        if self.detected_face_visualization:
            # Draw rectangle around the face (blue color, thickness 2 pixels)
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(image, (x + int(w / 2), y + int(h / 2)), 5, (0, 255, 0), -1)

        if self.additional_visualizations:
            # Draw line to center (blue color, thickness 1 pixel) - User preference
            cv2.line(image, (x + int(w / 2), y + int(h / 2)), (self.img_center_x, y + int(h / 2)), (0, 0, 255), 1)

            # Put text on top of the line (assuming white color, font scale 0.5)
            text = f"Offset: {offset}"
            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x = int((x + int(w / 2) + self.img_center_x) / 2 - text_size[0] / 2)
            text_y = y + int(h / 2) - 5
            cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Draw vertical center line (green color, thickness 1 pixel)
            image = cv2.line(image, (self.img_center_x, 0), (self.img_center_x, self.img_height), (0, 255, 0), 1)

        return image  # Return the image with additional visualizations (optional)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
