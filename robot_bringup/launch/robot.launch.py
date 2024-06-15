from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    camera_node = Node(
        package='camera_pkg',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters = [
            {"camera_source": "android"},
            {"ip": "192.168.0.104"},
            {"show_window": False}
        ]
    )
    
    object_detection_node = Node(
        package='camera_pkg',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        parameters = [
            {"detected_face_visualization": True},
            {"additional_visualizations": True},
            {"camera_window": False}
        ]
    )
    
    motor_controller_node = Node(
        package='motor_controller_pkg',
        executable='motor_controller_node',
        name='motor_controller_node',
        output='screen'
    )
    
    arduino_interface_node = Node(
        package='arduino_pkg',
        executable='arduino_interface_node',
        name='arduino_interface_node',
        output='screen'
    )
    
    ld.add_action(camera_node)
    ld.add_action(object_detection_node)
    ld.add_action(motor_controller_node)
    ld.add_action(arduino_interface_node)
    
    return ld
