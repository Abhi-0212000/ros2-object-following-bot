import os
import logging
from datetime import datetime
from logging.handlers import RotatingFileHandler

def get_file_modification_time(file_path):
    """Get the modification time of a file."""
    return os.path.getmtime(file_path)

class NodeLogger:
    def __init__(self, node_name, logging_level='INFO', max_logs=15, log_size=10*1024*1024):
        """
        Initialize the NodeLogger.
        
        Parameters:
            node_name (str): The name of the node.
            logging_level (str): The logging level (e.g., 'DEBUG', 'INFO').
            max_logs (int): Maximum number of log files to retain.
            log_size (int): Maximum size of each log file in bytes.
        """
        self.node_name = node_name
        self.log_dir = os.path.expanduser(f'~/robot_logs/{self.node_name}_logs/')
        self.max_logs = max_logs
        self.log_size = log_size
        self.logger = logging.getLogger(node_name)
        self.set_level(logging_level)

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        self._setup_handler()
        self.logger.info(f"=== Starting log for {node_name} ===")

    def set_level(self, level):
        """Set the logging level."""
        self.logger.setLevel(level)

    def _setup_handler(self):
        """Setup the log handler."""
        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        log_file = os.path.join(self.log_dir, f"{self.node_name}_log_{current_time}.txt")
        handler = RotatingFileHandler(log_file, maxBytes=self.log_size, backupCount=1)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self._clean_old_logs()

    def _clean_old_logs(self):
        """Clean up old logs to ensure we don't exceed the max_logs limit."""
        log_files = [f for f in os.listdir(self.log_dir) if os.path.isfile(os.path.join(self.log_dir, f))]
        
        # Ensure all log files exist before sorting
        log_files = [f for f in log_files if os.path.exists(os.path.join(self.log_dir, f))]
        
        log_files.sort(key=lambda f: get_file_modification_time(os.path.join(self.log_dir, f)))
        
        while len(log_files) > self.max_logs:
            os.remove(os.path.join(self.log_dir, log_files.pop(0)))

    def log_info(self, message):
        """Log an info message."""
        self.logger.info(message)

    def log_warning(self, message):
        """Log a warning message."""
        self.logger.warning(message)

    def log_error(self, message):
        """Log an error message."""
        self.logger.error(message)

    def log_debug(self, message):
        """Log a debug message."""
        self.logger.debug(message)

    def log_critical(self, message):
        """Log a critical message."""
        self.logger.critical(message)

    def close(self):
        """Close all handlers to ensure log file is saved properly."""
        handlers = self.logger.handlers[:]
        for handler in handlers:
            handler.close()
            self.logger.removeHandler(handler)
