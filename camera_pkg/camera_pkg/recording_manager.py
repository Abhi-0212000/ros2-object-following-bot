import os
from datetime import datetime
import subprocess

class RecordingManager:
    def __init__(self, node, record: bool, bag_directory: str = "~/robot_bags/bag_recordings", max_bags: int = 3):
        self.node = node
        self.bag_recording = False
        self.bag_directory = os.path.expanduser(bag_directory)
        self.max_bags = max_bags
        self.record = record
        self.recording_process = None

        if not os.path.exists(self.bag_directory):
            os.makedirs(self.bag_directory)
            self.node.get_logger().info(f"Directory created: {self.bag_directory}")
        else:
            self.node.get_logger().info(f"Using existing directory: {self.bag_directory}")

        if self.record:
            self.manage_recording()
            self.recording_timer = self.node.create_timer(120, self.manage_recording)

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
        self.bag_recording = True
        self.node.get_logger().info(f"Started recording: {bag_file_path}")

    def stop_recording(self):
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process.wait()
            self.bag_recording = False
            self.node.get_logger().info("Stopped recording")

    def cleanup_bag_files(self):
        bag_files = sorted([f for f in os.listdir(self.bag_directory) if f.endswith('.bag')],
                           key=lambda f: os.path.getmtime(os.path.join(self.bag_directory, f)))
        while len(bag_files) > self.max_bags:
            oldest_bag = bag_files.pop(0)
            os.remove(os.path.join(self.bag_directory, oldest_bag))
            self.node.get_logger().info(f"Deleted old bag file: {oldest_bag}")

    def destroy(self):
        self.node.get_logger().info("Destroying RecordingManager.")
        if self.bag_recording:
            self.stop_recording()
