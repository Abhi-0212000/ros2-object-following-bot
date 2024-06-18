To create a ROS2 package correctly (especially a utility package without any nodes), follow these steps:

### Step-by-Step Instructions for Creating a ROS2 Package

1. **Navigate to your ROS2 workspace**: 
   
   ```sh
   cd ~/robot_ws/src
   ```

2. **Create the package**:
   
   ```sh
   ros2 pkg create --build-type ament_python my_robot_utils
   ```

   This command creates a new ROS2 package with Python support. It will generate the necessary files and directories for the package.

3. **Verify Package Structure**: After running the command, you should have the following structure:

   ```
   my_robot_utils/
   ├── CMakeLists.txt
   ├── package.xml
   ├── resource/
   │   └── my_robot_utils
   ├── setup.cfg
   ├── setup.py
   ├── test/
   └── my_robot_utils/
       ├── __init__.py
       └── recording_manager.py
   ```

4. **Move your helper script**:
   
   Move your `recording_manager.py` script into the `my_robot_utils` directory:

   ```sh
   mv /path/to/your/recording_manager.py ~/robot_ws/src/my_robot_utils/my_robot_utils/
   ```

5. **Update setup.py**:

   Edit the `setup.py` file to include your module:

   ```python
   from setuptools import setup

   package_name = 'my_robot_utils'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       package_dir={'': 'src'},
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Package for helper functions and utilities',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [],
       },
   )
   ```

6. **Build the Package**:

   Navigate back to the root of your workspace and build the package:

   ```sh
   cd ~/robot_ws
   colcon build --packages-select my_robot_utils
   ```

7. **Source the Workspace**:

   After building, source your workspace:

   ```sh
   source ~/robot_ws/install/setup.bash
   ```

8. **Update `camera_pkg`**:

   In your `object_detection.py` script within `camera_pkg`, update the import statement to use the new package:

   ```python
   from my_robot_utils.recording_manager import RecordingManager

   # Rest of your code
   ```

9. **Add Dependency in `camera_pkg`**:

   Ensure that `camera_pkg`'s `package.xml` lists `my_robot_utils` as a dependency:

   ```xml
   <package format="3">
     <name>camera_pkg</name>
     <version>0.0.0</version>
     <description>The camera_pkg package</description>

     <maintainer email="your_email@example.com">your_name</maintainer>
     <license>Apache License 2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <build_depend>rclpy</build_depend>
     <exec_depend>rclpy</exec_depend>

     <build_depend>my_robot_utils</build_depend>
     <exec_depend>my_robot_utils</exec_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```
