import argparse
import os
from utils import msg_to_pil 
import time
import shutil

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy

IMAGE_TOPIC = "/usb_cam/image_raw"
TOPOMAP_IMAGES_DIR = "../topomaps/images"

class CreateTopomap(Node):
    def __init__(self, args):
        super().__init__("CREATE_TOPOMAP")
        
        # Logger
        self.logger = self.get_logger()
        
        # publish & subscribe
        self.subgoals_pub = self.create_publisher(Image, "/subgoals", 10)
        self.image_curr_msg = self.create_subscription(Image, IMAGE_TOPIC, self.callback_obs, 10)
        self.joy_sub = self.create_subscription(Joy, "joy", self.callback_joy, 10)
        self.dt = args.dt
        self.obs_img = None
        self.start_time = float("inf")
        self.count = 0
        
        self.topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
        if not os.path.isdir(self.topomap_name_dir):
            os.makedirs(self.topomap_name_dir)
            self.logger.info(f"Created directory: {self.topomap_name_dir}")
        else:
            self.logger.warning(f"Directory already exists. Cleaning: {self.topomap_name_dir}")
            self.remove_files_in_dir(self.topomap_name_dir)
        
        assert self.dt > 0, "dt must be positive"
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def remove_files_in_dir(self, dir_path: str):
        for f in os.listdir(dir_path):
            file_path = os.path.join(dir_path, f)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
                self.logger.info(f"Removed file or directory: {file_path}")
            except Exception as e:
                self.logger.error(f"Failed to delete {file_path}. Reason: {e}")

    def callback_obs(self, msg: Image):
        self.logger.debug("Received an image message.")
        self.obs_img = msg_to_pil(msg)
        
    def callback_joy(self, msg: Joy):
        if msg.buttons[0]:
            self.logger.info("Shutdown requested via joystick.")
            rclpy.shutdown()
            
    def timer_callback(self):
        if self.obs_img is not None:
            image_path = os.path.join(self.topomap_name_dir, f"{self.count}.png")
            self.obs_img.save(image_path)
            self.logger.info(f"Saved image {self.count}: {image_path}")
            self.count += 1
            self.start_time = time.time()
            self.obs_img = None
        
        if time.time() - self.start_time > 2 * self.dt:
            self.logger.warning(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
            rclpy.shutdown()

def main(args: argparse.Namespace):
    rclpy.init()
    node = CreateTopomap(args)
    node.get_logger().info("Registered with master node. Waiting for images...")
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
   
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Code to generate topomaps from the {IMAGE_TOPIC} topic"
    )
    parser.add_argument(
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topological map images in ../topomaps/images directory (default: topomap)",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=1.,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 3.0)",
    )
    args = parser.parse_args()
    main(args)
