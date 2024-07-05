### ObjectDetectionNode

1. **Initialization (`__init__` Method)**:
   - **Subscriptions and Publishers**:
     - Subscribes to the `camera_image` topic to receive images.
     - Publishes object positions to the `object_position` topic.
     - Publishes visualized images to the `visualized_image` topic (if recording is enabled).
   - **Parameters**:
     - `detected_face_visualization`: Toggles face detection visualization.
     - `additional_visualizations`: Toggles additional visualizations.
     - `camera_window`: Toggles the display of the camera feed window.
     - `record`: Toggles the recording of visualized images.
   - **Face Detection**:
     - Loads the Haar Cascade classifier for face detection.
   - **Image Dimensions**:
     - Sets image dimensions and calculates the image center.
   - **Timeout and Recording Management**:
     - Sets the timeout duration for object detection.
     - Initializes recording management, including a timer for periodic recordings and a directory for saving bag files.

2. **Image Callback (`detect_object_from_image` Method)**:
   - **Image Conversion**:
     - Converts the incoming ROS `Image` message to an OpenCV image.
   - **Face Detection**:
     - Detects faces in the image using the Haar Cascade classifier.
   - **Offset and Distance Calculation**:
     - Calculates the offset of the detected face from the image center.
     - Estimates the distance of the face from the camera.
   - **Publishing Object Position**:
     - Publishes the offset and distance.
   - **Drawing Visualizations**:
     - Draws bounding boxes and additional visualizations on the image.
   - **Displaying and Publishing Visualized Images**:
     - Displays the camera feed with visualizations if enabled.
     - Publishes the visualized image if recording is enabled.

3. **Offset and Distance Calculation Methods**:
   - **`calculate_offset` Method**:
     - Calculates the horizontal offset between the detected face's bounding box center and the image center.
   - **`estimate_distance` Method**:
     - Estimates the distance to the detected face based on the bounding box size and known camera parameters.

4. **Publishing Object Position (`publish_object_position` Method)**:
   - Publishes the calculated offset and distance as a `Float64MultiArray` message.

5. **Drawing Visualizations (`draw_visualizations` Method)**:
   - Draws bounding boxes around detected faces and additional visualizations like lines and text annotations based on user preferences.

6. **Recording Management (`manage_recording`, `start_recording`, `stop_recording`, `cleanup_bag_files` Methods)**:
   - **`manage_recording` Method**:
     - Called periodically by a timer to manage the recording process.
     - Starts or stops recording and cleans up old bag files.
   - **`start_recording` Method**:
     - Starts recording visualized images to a new bag file.
   - **`stop_recording` Method**:
     - Stops the current recording process.
   - **`cleanup_bag_files` Method**:
     - Deletes older bag files to ensure only a few recent recordings are kept.

### Main Function

The `main` function initializes the ROS 2 node, starts it, and handles cleanup on shutdown.

```python
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node.bag_recording:
            node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
```

### Summary

This implementation:

- Detects faces in incoming images and calculates their offset and distance from the camera center.
- Displays and optionally records the visualized images based on user parameters.
- Manages the recording process, ensuring only a few recent recordings are kept to save storage space.