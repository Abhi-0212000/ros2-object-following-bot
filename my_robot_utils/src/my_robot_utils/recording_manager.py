import os
from datetime import datetime
import subprocess
import shutil

class RecordingManager:
    def __init__(self, node, record: bool, bag_directory: str = "~/robot_logs/robot_bags/bag_recordings", max_bags: int = 3):
        self.node = node
        self.bag_recording = False
        self.bag_directory = os.path.expanduser(bag_directory)
        self.max_bags = max_bags
        self.record = record
        self.recording_process = None
        
        self.node.get_logger().info("in my_robot_utils pkg")
        if not os.path.exists(self.bag_directory):
            os.makedirs(self.bag_directory)
            self.node.get_logger().info(f"Directory created: {self.bag_directory}")
        else:
            self.node.get_logger().info(f"Using existing directory: {self.bag_directory}")

        if self.record:
            self.manage_recording()
            self.recording_timer = self.node.create_timer(150, self.manage_recording)
        #self.cleanup_bag_files()

    def manage_recording(self):
        self.node.get_logger().info("Manage recording called.")
        if self.record:
            self.node.get_logger().info("Recording flag is True.")
            if not self.bag_recording:
                self.node.get_logger().info("Starting new recording.")
                self.start_recording()
            else:
                self.node.get_logger().info("Stopping current recording.")
                self.stop_recording()
                self.node.get_logger().info("Starting new recording.")
                self.start_recording()
                self.node.get_logger().info("Cleaning up old bag files.")
                self.cleanup_bag_files()
        else:
            self.node.get_logger().info("Recording flag is False.")

    def start_recording(self):
        bag_file_path = os.path.join(self.bag_directory, f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.bag")
        self.recording_process = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_file_path, '--compression-mode', 
                                                   'file', '--compression-format', 'zstd', '/visualized_image'])
        # Adjust the compression format (zstd, bz2, lz4, etc.) based on your preference
        self.bag_recording = True
        self.node.get_logger().info(f"Started recording: {bag_file_path}")

    def stop_recording(self):
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.bag_recording = False
            self.node.get_logger().info("Stopped recording")

    def cleanup_bag_files(self):
        # List directories in the bag_directory
        bag_directories = [d for d in os.listdir(self.bag_directory) if os.path.isdir(os.path.join(self.bag_directory, d))]
        
        # Sort directories by modification time (oldest to newest)
        bag_directories.sort(key=lambda d: os.path.getmtime(os.path.join(self.bag_directory, d)))
        
        # Remove old directories if the number of directories exceeds max_bags
        while len(bag_directories) > self.max_bags:
            oldest_bag_directory = bag_directories.pop(0)
            self.remove_bag_directory(oldest_bag_directory)

    def remove_bag_directory(self, directory_name):
        directory_path = os.path.join(self.bag_directory, directory_name)
        try:
            shutil.rmtree(directory_path)
            self.node.get_logger().info(f"Deleted old bag directory: {directory_name}")
        except OSError as e:
            self.node.get_logger().error(f"Error deleting bag directory {directory_name}: {e}")
            
    def destroy(self):
        self.node.get_logger().info("Destroying RecordingManager.")
        if self.bag_recording:
            self.stop_recording()
            self.cleanup_bag_files()
