### Updated Software Architecture

1. **Camera Node**: This node will handle the camera input.
   - **Node Name**: `camera_node`
   - **Responsibilities**:
     - Access the camera (using the Android phone as an IP camera).
     - Read a frame and publish the frames
   - **Topic**:
     - **Publish**: `/camera_image` (sensor_msgs/Image)
   - **Parameters**:
      - `camera_source`, `ip`, `show_window`

2. **Object Detection Node**: This node handles the object detection and offset calculations.
   - **Node Name**: `object_detection_node`
   - **Responsibilities**:
     - Read a frame and detect the object and calculate the horizontal offset between detectd obj center and image center, estimated distance to object.
   - **Topic**:
     - **Publish**: `/object_position` (std_msgs/msg/Float64MultiArray) [offset, distance]
     - **Subscribe**: `/camera_image` (sensor_msgs/Image) for object detection
   - **Parameters**:
      - `detected_face_visualization`, `additional_visualizations`, `camera_window`

3. **Motor Controller Node**: This node will control the motors based on the detected object offset value.
   - **Node Name**: `motor_controller_node`
   - **Responsibilities**:
     - Send motor speed based on offset between center's and distance from camera to object.
   - **Topics**:
     - **Publish**: `/cmd_vel` (geometry_msgs/Twist) to send commands to motors
     - **Subscribe**: `/object_position` (std_msgs/msg/Float64MultiArray) to receive detected object offset, distance

4. **Arduino Node**: Interface with the Arduino to control the motors.
   - **Node Name**: `arduino_node`
   - **Responsibilities**:
     - Interface with the Arduino to control stepper motors.
   - **Topics**:
     - **Subscribe**: `/cmd_vel` (geometry_msgs/Twist) to receive motor commands

### Suggested Package and Topic Names

- **Workspace Name**: `robot_ws`
- **Packages**:
  - `camera_pkg`: It contains `camera_node` and `object_detection_node`
  - `motor_controller_pkg`: It contains `motor_controller_node`
  - `arduino_pkg`: It contains `arduino_interface_node`
  - `robot_bringup`: It contains launch file to start all the nodes.
- **Topics**:
  - `/camera_image` (sensor_msgs/Image)
  - `/object_position` (std_msgs/msg/Float64MultiArray) i.e [offset, distance]
  - `/cmd_vel` (geometry_msgs/Twist)

